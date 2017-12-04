#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;
using namespace std;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

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

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

// convert from map frame to car frame
void map2car(vector<double> x_in, vector<double> y_in, vector<double> &x_out, vector<double> &y_out, double psi, double x0, double y0) {
  // @input: x_in, y_in: coordinates in first frame
  // @input/output: x_out, y_out : coordinates in second frame
  // @input: psi : yaw (rad)
  // @input: x0, y0 ycoordinates of center of second frame in first frame
  for (unsigned int i = 0; i<x_in.size(); ++i) {
    x_out.push_back(std::cos(psi)*(x_in[i]-x0)+std::sin(psi)*(y_in[i]-y0));
    y_out.push_back(-std::sin(psi)*(x_in[i]-x0)+std::cos(psi)*(y_in[i]-y0));
  }
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
          const double Lf = 2.67;
          // define latency in ms
          const unsigned int t_latency = 100;
          double latency = t_latency/1000.0;   // convert in s

          // declare state variables
          double px_car;  // x-coordinate in car frame
          double py_car;  // y-coordinate in car frame
          double psi_car; // yaw in car frame
          double epsi;    // yaw error
          double cte;     // cross track error
          double steer_value;
          double throttle_value;

          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          double delta = j[1]["steering_angle"];
          double acc = j[1]["throttle"];

          // input corrections
          //v*=0.44704;  // conversion from mph to m/s
          delta *=-1;   // take opposite sign as positive commands means right turn

          // convert reference trajectory coordinates from map to car frame
          unsigned int n_pts = ptsx.size();
          vector<double> ptsx_car;
          vector<double> ptsy_car;
          map2car(ptsx, ptsy, ptsx_car, ptsy_car, psi, px, py);

          // fit a polynomial to the ref x and y coordinates
          int polyorder = 2;
          // ptsx, ptsy format conversion from std::vector to eigen::vector
          Eigen::VectorXd ptsx_(n_pts);
          Eigen::VectorXd ptsy_(n_pts);
          for (unsigned int i = 0; i<n_pts; ++i) {
            ptsx_[i] = ptsx_car[i];
            ptsy_[i] = ptsy_car[i];
          }
          auto coeffs = polyfit(ptsx_, ptsy_, polyorder) ;

          // determine state values
          px_car = 0;
          py_car = 0;
          psi_car = 0;

          // compute derivative of polynomial at origin
          double f_prime_x;
          if (polyorder == 1) {
            f_prime_x = coeffs[1];
            //cout<<"coeff="<<coeffs[0]<<" "<<coeffs[1]<<endl;
            //cout<<"f_prim = "<<f_prime_x<<"  /  ";
          }
          else if (polyorder == 2) {
            f_prime_x = coeffs[1] + 2*coeffs[2]*px_car;
            //cout<<"coeff="<<coeffs[0]<<" "<<coeffs[1]<<" "<<coeffs[2]<<endl;
            //cout<<"f_prim = "<<f_prime_x<<"  /  ";
          }
          else if (polyorder == 3) {
            f_prime_x = coeffs[1] + 2*coeffs[2]*px_car + 3*coeffs[3]*px_car*px_car;
            //cout<<"coeff="<<coeffs[0]<<" "<<coeffs[1]<<" "<<coeffs[2]<<" "<<coeffs[3]<<endl;
            //cout<<"f_prim = "<<f_prime_x<<"  /  ";
          }
          else {
            std::cout << "ERROR: POLYNOMIAL ORDER MUST BE between 1 and 3" << endl;
          }
          epsi = psi_car - atan(f_prime_x);
          cte = polyeval(coeffs, px_car) - py_car;

          // take into account latency
          // propagate current state by latency time
          double px_car_lat = px_car + v*latency*cos(psi_car);  // = v*latency
          double py_car_lat = py_car + v*latency*sin(psi_car);  // = 0
          double psi_car_lat = psi_car + v*delta*latency/Lf;    // = v*delta*latency/Lf
          double v_lat = v + acc*latency; // neglect latency
          double cte_lat = cte + v*sin(epsi)*latency;           // = polyeval(coeffs,px_car)+v*sin(epsi)*latency
          double epsi_lat = epsi + v*delta*latency/Lf;          // = -atan(f_prime_x)+v*delta*latency/Lf
          //cout <<"px_car = " << px_car_lat <<" / py_car = " << py_car_lat <<" / psi_car = " << psi_car_lat << " / epsi = "<<epsi_lat<<" / cte="<<cte_lat<<endl;

          Eigen::VectorXd state(6);
          state << px_car_lat, py_car_lat, psi_car_lat, v_lat, cte_lat, epsi_lat;

          // solve MPC problem
          auto vars = mpc.Solve(state, coeffs);
          // vars contains the next N waypoints (x,y) plus first actuator values (x0,..,xN-1,y0,..,yN-1,delta1,a1]
          unsigned int N = (vars.size()-2)/2;

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          steer_value = -vars[2*N]/deg2rad(25);
          throttle_value = vars[2*N+1];
          cout<<"steer_value="<<steer_value<<endl;
          cout<<"throttle_value="<<throttle_value<<endl;

          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory
          vector<double> mpc_x;
          vector<double> mpc_y;
          for (unsigned int i=0; i<N; ++i) {
            mpc_x.push_back(vars[i]);
            mpc_y.push_back(vars[i+N]);
          }
          /*
          for (unsigned int i=0; i<N; ++i) {
            cout<<"Pred traj = "<<mpc_x[i]<<" / "<<mpc_y[i]<<endl;
          }*/

          //Display the reference trajectory
          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line
          vector<double> ref_x;
          vector<double> ref_y;
          double x;
          double y;
          for (unsigned int i=0; i<n_pts; ++i) {
            x = 7*i+1;
            y = polyeval(coeffs,x);
            ref_x.push_back(x);
            ref_y.push_back(y);
            //cout<<"Ref traj = "<<x<<" / "<<y<<endl;
          }
          msgJson["next_x"] = ref_x;
          msgJson["next_y"] = ref_y;
          //Display the MPC trajectory
          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line
          msgJson["mpc_x"] = mpc_x;
          msgJson["mpc_y"] = mpc_y;

          cout<<endl;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(t_latency));
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
