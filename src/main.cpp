#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }
  int lane = 1;
  double ref_vel = 0, prev_ref_vel = 0;
  int iter =0;
  bool switchinglanes = false;

  h.onMessage([&lane,&ref_vel,&prev_ref_vel,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,&iter,&switchinglanes]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];
          double next_s, next_d;
          double ref_x,ref_y, ref_yaw;
          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];
          int prev_size = previous_path_x.size();

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          vector<vector<double>>sensor_fusion = j[1]["sensor_fusion"];
  

          json msgJson;
          iter++;
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          vector<double> ptsx, ptsy;
          ref_x = car_x;
          ref_y = car_y;
          ref_yaw = deg2rad(car_yaw);
          if(prev_size>0){
            car_s = end_path_s;
          }
          
          
          bool vehicle_ahead = false;
          vector<bool> safetoswitch = {true,true,true};
          
          if(switchinglanes==false){

            for(int i=0;i<sensor_fusion.size();i++){
              bool inlane = false;
              float veh_d = sensor_fusion[i][6];
              
              if(veh_d>2+(lane*4)-2 && veh_d<2+(lane*4)+2){
                double veh_vx = sensor_fusion[i][3];
                double veh_vy = sensor_fusion[i][4];
                double veh_speed = sqrt((veh_vx*veh_vx) + (veh_vy*veh_vy));
                double veh_s = sensor_fusion[i][5];
                double future_veh_s = veh_s + (double)prev_size*0.02*veh_speed;
                bool inlane = true;
                safetoswitch[lane] = false;
                if(car_s < veh_s && future_veh_s-car_s<30){
                  ref_vel = (ref_vel<1.05*veh_speed)? veh_speed : ref_vel - 0.2;
                  vehicle_ahead = true;
                }  
              }else{
                
                double veho_vx = sensor_fusion[i][3];
                double veho_vy = sensor_fusion[i][4];
                double veho_speed = sqrt((veho_vx*veho_vx) + (veho_vy*veho_vy));
                double veho_s = sensor_fusion[i][5];
                int lane_dir = (car_d<veh_d) ? 1 : -1;
                int lane_s = (abs(car_d-veh_d) >6) ?  lane_dir*2 +lane : lane+lane_dir;
                double future_veho_s = veho_s + (double)prev_size*0.02*veho_speed;
                if(future_veho_s-car_s<30 && future_veho_s-car_s>-20){          
                  std::cout<<"enterd "<<lane_s<<std::endl;
                  if (lane_s == 1)
                    safetoswitch = {false,false,false};
                  else safetoswitch[lane_s] = false;
               
                }       
              }
            }
          }else if(car_d>2+(lane*4)-1 && car_d<2+(lane*4)+1){
            switchinglanes = false;          
          }
          
          if(vehicle_ahead==true){
            std::cout<<"entered lane switch"<<std::endl;
            vector<int> p = {0,1,2,1,0};
          
            for(int i=lane;i<5;i++){
              if(safetoswitch[p[i]]){
                lane = p[i];
                switchinglanes = true;
                break;
              }
            }
          }else{
            ref_vel = (ref_vel<47) ? ref_vel+1.1: 47;  
          }
          prev_ref_vel = ref_vel;
          
          
          if (prev_size<2){
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);
            
            ptsx.push_back(prev_car_x);
            ptsy.push_back(prev_car_y);
             
            ptsx.push_back(car_x);
            ptsy.push_back(car_y);
          
          }else{
            double prev_car_x = previous_path_x[prev_size-2];
            double prev_car_y = previous_path_y[prev_size-2];
            
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];
            ref_yaw = atan2(ref_y - prev_car_y,ref_x - prev_car_x);
            ptsx.push_back(prev_car_x);
            ptsy.push_back(prev_car_y);
             
            ptsx.push_back(ref_x);
            ptsy.push_back(ref_y);
            
          }
         
          vector<double> getwp0 = getXY(car_s+30,2+(lane*4),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector<double> getwp1 = getXY(car_s+40,2+(lane*4),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector<double> getwp2 = getXY(car_s+50,2+(lane*4),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          
          ptsx.push_back(getwp0[0]);
          ptsx.push_back(getwp1[0]);
          ptsx.push_back(getwp2[0]);
          
          ptsy.push_back(getwp0[1]);
          ptsy.push_back(getwp1[1]);
          ptsy.push_back(getwp2[1]);
          for(int i=0; i<ptsx.size(); i++){
            double shift_x = ptsx[i]-ref_x;
            double shift_y = ptsy[i]-ref_y;
            
            ptsx[i] = (shift_x*cos(0-ref_yaw) - shift_y*sin(0-ref_yaw));
            ptsy[i] = (shift_x*sin(0-ref_yaw) + shift_y*cos(0-ref_yaw));
            
          }
          tk::spline s;
          s.set_points(ptsx,ptsy);
          
          for(int i=0;i<previous_path_x.size();i++){
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt((target_x*target_x) + (target_y*target_y));
          
          double x_add_on = 0;
          
          for(int i=1;i<= 50-previous_path_x.size();i++){
            double N = target_dist/(0.02*ref_vel/2.24);
            double x_point = x_add_on + target_x/N;
            double y_point = s(x_point);
            
            x_add_on = x_point;
            double x_ref = x_point;
            double y_ref = y_point;
            
            x_point = (x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw));
            y_point = (x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw));
            
            x_point += ref_x;
            y_point += ref_y;
            
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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