#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "tools.h"
#include "json.hpp"

using namespace Eigen;

// for convenience
using json = nlohmann::json;


const int latency = 100;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}


int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          double delta = j[1]["steering_angle"];
          double a = j[1]["throttle"];

          // convert waypoints to car coordinates
          VectorXd ptsxCar(ptsx.size());
          VectorXd ptsyCar(ptsy.size());
          for(size_t i = 0; i < ptsx.size(); ++i) {
            double dx = ptsx[i] - px;
            double dy = ptsy[i] - py;
            ptsxCar(i) = dx * cos(psi) + dy * sin(psi);
            ptsyCar(i) = dy * cos(psi) - dx * sin(psi);
          }

          // fit a polynomial to the above x and y coordinates in car coordinates
          auto coeffs = polyfit(ptsxCar, ptsyCar, 3);

         // car is at (0,0) inand thus simplified errors
          double cte = polyeval(coeffs, 0.0);
          double epsi = -atan(coeffs[1]);

          // handle latency
          // assume the car keeps moving for the latency duration before the new actuations kick in
          double dtLatency = latency * 0.001; // ms to s
          double pxPred = 0.0 + v * dtLatency; // cos(0) == 1
          double pyPred = 0.0; // sin(0) == 0
          double psiPred = 0.0 - v * delta / Lf * dtLatency;
          double vPred = v + a * dtLatency;
          double ctePred = cte + v * sin(epsi) * dtLatency;
          double epsiPred = epsi - v * delta / Lf * dtLatency;

          Eigen::VectorXd state(6);
          state << pxPred, pyPred, psiPred, vPred, ctePred, epsiPred;

          // calculate modeled states and actuators
          vector<double> vars = mpc.Solve(state, coeffs);
          double steer_value = vars[0] / (deg2rad(25) * Lf); // map to [-1, 1] and scale with turn radius
          double throttle_value = vars[1];

          // build message
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;


          //Display the MPC predicted trajectory
          size_t N = (vars.size() - 2)/2;
          vector<double> mpc_x_vals(N);
          vector<double> mpc_y_vals(N);
          std::copy(vars.begin()+2, vars.begin()+2+N, mpc_x_vals.begin());
          std::copy(vars.begin()+2+N, vars.end(), mpc_y_vals.begin());

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          for(int i = 1; i <= 20; ++i) {
              next_x_vals.push_back(i*3);
              next_y_vals.push_back(polyeval(coeffs, i*3));
            }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(latency));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
