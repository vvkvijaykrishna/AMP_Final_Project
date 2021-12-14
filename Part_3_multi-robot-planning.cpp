#include <iostream>
#include <vector>
#include <time.h>
#include <fstream>
#include <random>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <chrono>

using namespace std;

float pi=3.14159;
float theta_min = 0.0;
float theta_max = 6.283;
float pgoal = 0.05;

//arm lengths
float l1 = 3.0;
float l2 = 2.0;
float l3 = 2.0;

//start and end points
float xs1 = -2;  //--------> start points of manipulator 1
float ys1 = 1;
float zs1 = 0;

float xs2 = 5;  //--------> start points of manipulator 2
float ys2 = 1;
float zs2 = 0;

float xs3 = 0;  //--------> start points of manipulator 3
float ys3 = 1;
float zs3 = 5;

float xe1 = 1;   //--------> end points of manipulator 1
float ye1 = 2;
float ze1 = 2;

float xe2 = 1;   //--------> end points of manipulator 2
float ye2 = 2;
float ze2 = 1;

float xe3 = 2;   //--------> end points of manipulator 3
float ye3 = 2;
float ze3 = 1;

float xn1 = 0;   //----------> location of base of manipulator 2 with respect to the origin
float yn1 = 0;
float zn1 = 0;

float xn2 = 3;   //----------> location of base of manipulator 2 with respect to the origin
float yn2 = 0;
float zn2 = 0;

float xn3 = 0;   //----------> location of base of manipulator 3 with respect to the origin
float yn3 = 0;
float zn3 = 3;

vector <float> start_point_man1 { xs1, ys1, zs1 };
vector <float> start_point_man2 { xs2, ys2, zs2 };
vector <float> start_point_man3 { zs3, zs3, zs3 };
vector <float> end_point_man1 { xe1, ye1, ze1 };
vector <float> end_point_man2 { ze2, ze2, ze2 };
vector <float> end_point_man3 { xe3, ye3, ze3 };

//function definition
vector <float> get_thetas (const vector <float> &point, const int &manipulator_number);  //-----> inverse kinematics

struct state{
    float theta1;
    float theta2;
    float theta3;
    float theta4;
    float theta5;
    float theta6;
    float theta7;
    float theta8;
    float theta9;
    
    float x1;
    float y1;
    float z1;
    
    float x2;
    float y2;
    float z2 ;
    
    float x3;
    float y3;
    float z3;
    
    float x4;
    float y4;
    float z4;
    
    float x5;
    float y5;
    float z5;
    
    float x6;
    float y6;
    float z6;
    
    float x7;
    float y7;
    float z7;

    float x8;
    float y8;
    float z8;
    
    float x9;
    float y9;
    float z9;
    float time;
    
    int state_number;
    int previous_state;
    
    state(float mtheta1, float mtheta2, float mtheta3, float mtheta4, float mtheta5, float mtheta6, float mtheta7, float mtheta8, float mtheta9){
        theta1 = mtheta1;
        theta2 = mtheta2;
        theta3 = mtheta3;
        theta4 = mtheta4;
        theta5 = mtheta5;
        theta6 = mtheta6;
        theta7 = mtheta7;
        theta8 = mtheta8;
        theta9 = mtheta9;
        
        x1 = 0;
        y1 = l1;
        z1 = 0;
    
        x2 = l2*sin(theta2)*cos(theta1);
        y2 = l1 - l2*cos(theta2);
        z2 = l2*sin(theta2)*sin(theta1);
    
        x3 = ( l2*sin(theta2) - l3*sin(theta2+theta3) )*cos(theta1);
        y3 = l1 - l2*cos(theta2) + l3*cos(theta2+theta3);
        z3 = ( l2*sin(theta2) - l3*sin(theta2+theta3) )*sin(theta1);
        
        x4 = 0 + xn2;
        y4 = l1 + yn2;
        z4 = 0 +zn2;
    
        x5 = l2*sin(theta5)*cos(theta4) + xn2;
        y5 = l1 - l2*cos(theta5) + yn2;
        z5 = l2*sin(theta5)*sin(theta4) + zn2;
    
        x6 = ( l2*sin(theta5) - l3*sin(theta5+theta6) )*cos(theta4) + xn2;
        y6 = l1 - l2*cos(theta5) + l3*cos(theta5+theta6) + yn2;
        z6 = ( l2*sin(theta5) - l3*sin(theta5+theta6) )*sin(theta4) + zn2;
        
        x7 = 0 + zn3;
        y7 = l1 + zn3;
        z7 = 0 +zn3;

        x8 = l2*sin(theta8)*cos(theta7) + zn3;
        y8 = l1 - l2*cos(theta8) + zn3;
        z8 = l2*sin(theta8)*sin(theta7) + zn3;
    
        x9 = ( l2*sin(theta8) - l3*sin(theta8+theta9) )*cos(theta7) + zn3;
        y9 = l1 - l2*cos(theta8) + l3*cos(theta8+theta9) + zn3;
        z9 = ( l2*sin(theta8) - l3*sin(theta8+theta9) )*sin(theta7) + zn3;
        
        state_number = INT_MAX;
        previous_state = INT_MAX;
        
        time = 0;
    }
    
    state(){
        theta1 = 0.0;
        theta2 = 0.0;
        theta3 = 0.0;
        theta4 = 0.0;
        theta5 = 0.0;
        theta6 = 0.0;
        theta7 = 0.0;
        theta8 = 0.0;
        theta9 = 0.0;
        
        x1 = 0;
        y1 = l1;
        z1 = 0;
    
        x2 = l2*sin(theta2)*cos(theta1);
        y2 = l1 - l2*cos(theta2);
        z2 = l2*sin(theta2)*sin(theta1);
    
        x3 = ( l2*sin(theta2) - l3*sin(theta2+theta3) )*cos(theta1);
        y3 = l1 - l2*cos(theta2) + l3*cos(theta2+theta3);
        z3 = ( l2*sin(theta2) - l3*sin(theta2+theta3) )*sin(theta1);
        
        x4 = 0 + xn2;
        y4 = l1 + yn2;
        z4 = 0 +zn2;
    
        x5 = l2*sin(theta5)*cos(theta4) + xn2;
        y5 = l1 - l2*cos(theta5) + yn2;
        z5 = l2*sin(theta5)*sin(theta4) + zn2;
    
        x6 = ( l2*sin(theta5) - l3*sin(theta5+theta6) )*cos(theta4) + xn2;
        y6 = l1 - l2*cos(theta5) + l3*cos(theta5+theta6) + yn2;
        z6 = ( l2*sin(theta5) - l3*sin(theta5+theta6) )*sin(theta4) + zn2;
        
        x7 = 0 + xn3;
        y7 = l1 + yn3;
        z7 = 0 +zn3;
    
        x8 = l2*sin(theta5)*cos(theta4) + xn3;
        y8 = l1 - l2*cos(theta5) + yn3;
        z8 = l2*sin(theta5)*sin(theta4) + zn3;
    
        x9 = ( l2*sin(theta5) - l3*sin(theta5+theta6) )*cos(theta4) + xn3;
        y9 = l1 - l2*cos(theta5) + l3*cos(theta5+theta6) + yn3;
        z9 = ( l2*sin(theta5) - l3*sin(theta5+theta6) )*sin(theta4) + zn3;
        
        state_number = INT_MAX;
        previous_state = INT_MAX;
        
        time = 0;
    }
    
        //position of the end points (x1,y1,z1)   (x2,y2,z2)  (x3,y3,z3)
    
    void display_state(){
        cout << "\nStates-------------- at time: " << time <<"\n";
        cout << "Angles(manipulator 1) are: (" << theta1 << "\t" <<theta2 << "\t" << theta3 << ")\n" <<endl;
        cout << "Angles(manipulator 2) are: (" << theta4 << "\t" <<theta5 << "\t" << theta6 << ")\n" <<endl;
        cout << "Angles(manipulator 3) are: (" << theta7 << "\t" <<theta8 << "\t" << theta9 << ")\n" <<endl;
        cout << "Point positions are :\n" << "Point 1: ( " << x1 << " , " << y1 << " , " << z1 << " )" <<
            "\tPoint 2: ( " << x2 << " , " << y2 << " , " << z2 << " )\t";
        cout << "Point 3: ( " << x3 << " , " << y3 << " , " << z3 << " )\n" << endl;
        cout << "Point 4: ( " << x4 << " , " << y4 << " , " << z4 << " )" << endl;
        cout << "Point 5: ( " << x5 << " , " << y5 << " , " << z5 << " )" << endl;
        cout << "Point 6: ( " << x6 << " , " << y6 << " , " << z6 << " )\n" << endl;
        cout << "Point 7: ( " << x7 << " , " << y7 << " , " << z7 << " )" << endl;
        cout << "Point 8: ( " << x8 << " , " << y8 << " , " << z8 << " )" << endl;
        cout << "Point 9: ( " << x9 << " , " << y9 << " , " << z9 << " )\n" << endl;
        
    }
    
//    int check_state( vector <vector <vector <float>>> plane_parameters ){
//       if(link1_link3_intersection()==1){
//           return 0;                  //indicates intersection
//       }
//       else{
//          for( int i=0; i<plane_parameters.size; i++){
//              for( int j=0; j<plane_parameters.at(i).size; j++){
//                  float s1 = plane_parameters.at(i).at(j).(0)*x2 + plane_parameters.at(i).at(j).(1)*y2 + plane_parameters.at(i).at(j).(2)*z2 + plane_parameters.at(i).at(j).(3);
//                  float s2 = plane_parameters.at(i).at(j).(0)*x3 + plane_parameters.at(i).at(j).(1)*y3 + plane_parameters.at(i).at(j).(2)*z3 + plane_parameters.at(i).at(j).(3);
//                  if ( (s1*s2) < 0 )
//                  //check for intersection of line on plane
//                  {
//                  }
//                  else
//                      return 0;
//              }
//          } 
//       }
//    }

    float projection_vector(const vector <float> &a, const vector <float> &b){
        float x{};
        for(int i=0; i< a.size(); i++){
            x += (a.at(i)) * (b.at(i)) ;
        }
        
        float y{};
        for(int i=0; i< a.size(); i++){
            y += a.at(i)*a.at(i);
        }
        y = sqrt(y);
        
        x /= y;
        return x;
    }
    
    float euclid_dist(const vector <float> &a, const vector <float> &b){
        float x{};
        for(int i=0; i< a.size(); i++){
            x += (a.at(i) - b.at(i)) * (a.at(i) - b.at(i)) ;
        }
        x = sqrt(x);
        return x;
    }
    
    int intersection_cond_1d( const vector <float> &a, const vector <float> &b2, const vector <float> &b3, const vector <float> &p1, const vector <float> &p2){
            float d1 = projection_vector(a,b3); 
            float d2 = projection_vector(a,b2);
            float d = euclid_dist (p1,p2);
            if( ( (0<d1) && (d1<d) ) || ( (0<d2) && (d2<d) ) || ( 0>d1 && (d2>d) ) || ( 0>d2 && (d1>d) ) )
                return 1;       //intersection
            else 
                return 0;       //valid
    }
    
    int isstate_goal (){  //change the goal function
         if( (euclid_dist( vector<float>{x3,y3,z3} , vector<float>{xe1,ye1,ze1} ) <=0.3) && (euclid_dist( vector<float>{x6,y6,z6} , vector<float>{xe2,ye2,ze2} ) <=0.3) &&
                (euclid_dist( vector<float>{x9,y9,z9} , vector<float>{xe3,ye3,ze3} ) <=0.3) )
                 return 1;
         else
             return 0;
    }
    
    int ismanip_1_goal (){ 
         if( euclid_dist( vector<float>{x3,y3,z3} , vector<float>{xe1,ye1,ze1} ) <=0.3 )
                 return 1;
         else
             return 0;
    }
    
    int ismanip_2_goal (){ 
         if( euclid_dist( vector<float>{x6,y6,z6} , vector<float>{xe2,ye2,ze2} ) <=0.3 )
                 return 1;
         else
             return 0;
    }
    
    int ismanip_3_goal (){ 
         if( euclid_dist( vector<float>{x9,y9,z9} , vector<float>{xe3,ye3,ze3} ) <=0.3 )
                 return 1;
         else
             return 0;
    }
    
    void randomize_state ( const vector <float> &man1_goal, const vector <float> &man2_goal, const vector <float> &man3_goal, const vector <int> manip_goal){
        float prob = rand() % 100 + 0;
        if( (prob/100) <= pgoal )
         {
            theta1=man1_goal.at(0); theta2=man1_goal.at(1); theta3=man1_goal.at(2);
            theta4=man2_goal.at(0); theta5=man2_goal.at(1); theta6=man2_goal.at(2);
            theta7=man3_goal.at(0); theta8=man3_goal.at(1); theta9=man3_goal.at(2);
         }
        else{
            theta1 = rand() % static_cast<int>( theta_max*1000 +1 - theta_min*1000 ) + static_cast<int>(theta_min*1000); theta1/=1000;
            theta2 = rand() % static_cast<int>( theta_max*1000 +1 - theta_min*1000 ) + static_cast<int>(theta_min*1000); theta2/=1000;
            theta3 = rand() % static_cast<int>( theta_max*1000 +1 - theta_min*1000 ) + static_cast<int>(theta_min*1000); theta3/=1000;
            theta4 = rand() % static_cast<int>( theta_max*1000 +1 - theta_min*1000 ) + static_cast<int>(theta_min*1000); theta4/=1000;
            theta5 = rand() % static_cast<int>( theta_max*1000 +1 - theta_min*1000 ) + static_cast<int>(theta_min*1000); theta5/=1000;
            theta6 = rand() % static_cast<int>( theta_max*1000 +1 - theta_min*1000 ) + static_cast<int>(theta_min*1000); theta6/=1000;
            theta7 = rand() % static_cast<int>( theta_max*1000 +1 - theta_min*1000 ) + static_cast<int>(theta_min*1000); theta7/=1000;
            theta8 = rand() % static_cast<int>( theta_max*1000 +1 - theta_min*1000 ) + static_cast<int>(theta_min*1000); theta8/=1000;
            theta9 = rand() % static_cast<int>( theta_max*1000 +1 - theta_min*1000 ) + static_cast<int>(theta_min*1000); theta9/=1000;
        }
        
        if ( manip_goal.at(0) == 1 ){
            theta1=man1_goal.at(0); theta2=man1_goal.at(1); theta3=man1_goal.at(2);
        }
        
        if ( manip_goal.at(1) == 1 ){
            theta4=man2_goal.at(0); theta5=man2_goal.at(1); theta6=man2_goal.at(2);
        }
        
        if ( manip_goal.at(2) == 1 ){
            theta7=man3_goal.at(0); theta8=man3_goal.at(1); theta9=man3_goal.at(2);
        }
        
        x1 = 0;
        y1 = l1;
        z1 = 0;
    
        x2 = l2*sin(theta2)*cos(theta1);
        y2 = l1 - l2*cos(theta2);
        z2 = l2*sin(theta2)*sin(theta1);
    
        x3 = ( l2*sin(theta2) - l3*sin(theta2+theta3) )*cos(theta1);
        y3 = l1 - l2*cos(theta2) + l3*cos(theta2+theta3);
        z3 = ( l2*sin(theta2) - l3*sin(theta2+theta3) )*sin(theta1);
        
        x4 = 0 + xn2;
        y4 = l1 + yn2;
        z4 = 0 +zn2;
    
        x5 = l2*sin(theta5)*cos(theta4) + xn2;
        y5 = l1 - l2*cos(theta5) + yn2;
        z5 = l2*sin(theta5)*sin(theta4) + zn2;
    
        x6 = ( l2*sin(theta5) - l3*sin(theta5+theta6) )*cos(theta4) + xn2;
        y6 = l1 - l2*cos(theta5) + l3*cos(theta5+theta6) + yn2;
        z6 = ( l2*sin(theta5) - l3*sin(theta5+theta6) )*sin(theta4) + zn2;
        
        x7 = 0 + xn3;
        y7 = l1 + yn3;
        z7 = 0 +zn3;
    
        x8 = l2*sin(theta5)*cos(theta4) + xn3;
        y8 = l1 - l2*cos(theta5) + yn3;
        z8 = l2*sin(theta5)*sin(theta4) + zn3;
    
        x9 = ( l2*sin(theta5) - l3*sin(theta5+theta6) )*cos(theta4) + xn3;
        y9 = l1 - l2*cos(theta5) + l3*cos(theta5+theta6) + yn3;
        z9 = ( l2*sin(theta5) - l3*sin(theta5+theta6) )*sin(theta4) + zn3;
        
    }
    
    int check_state ( const vector <vector <vector <float>>> &plane_parameters , const int &a){         // function which takes the obstacle vector and tells whether the state is valid or not
        
       if( a == 1 ){
        if( (link1_link3_intersection() == 1) ){
           //cout<<"\n\nLink 1 and link 3 intersect here: \n";
           //display_state();//-------------------------------------------------display_state
           return 1;                  //indicates intersection
       }
       
       else{
           //check for each obstalce
           int count_ok{};
           for( int i=0; i<plane_parameters.size() ; i++ ){   //obstacle vector and not plne parameters
               int count = 0;
               vector <float> a1 { (plane_parameters.at(i).at(5).at(0) - plane_parameters.at(i).at(6).at(0)) , (plane_parameters.at(i).at(5).at(1) - plane_parameters.at(i).at(6).at(1)) , (plane_parameters.at(i).at(5).at(2) - plane_parameters.at(i).at(6).at(2)) }; 
               vector <float> a2 { (plane_parameters.at(i).at(7).at(0) - plane_parameters.at(i).at(6).at(0)) , (plane_parameters.at(i).at(7).at(1) - plane_parameters.at(i).at(6).at(1)) , (plane_parameters.at(i).at(7).at(2) - plane_parameters.at(i).at(6).at(2)) };
               vector <float> a3 { (plane_parameters.at(i).at(2).at(0) - plane_parameters.at(i).at(6).at(0)) , (plane_parameters.at(i).at(2).at(1) - plane_parameters.at(i).at(6).at(1)) , (plane_parameters.at(i).at(2).at(2) - plane_parameters.at(i).at(6).at(2)) };
               vector <float> b3 { x3 - plane_parameters.at(i).at(6).at(0), y3 - plane_parameters.at(i).at(6).at(1) , z3 - plane_parameters.at(i).at(6).at(2) };
               vector <float> b2 { x2 - plane_parameters.at(i).at(6).at(0), y2 - plane_parameters.at(i).at(6).at(1) , z2 - plane_parameters.at(i).at(6).at(2) };
               vector <float> b1 { x1 - plane_parameters.at(i).at(6).at(0), y1 - plane_parameters.at(i).at(6).at(1) , z1 - plane_parameters.at(i).at(6).at(2) };
               
               count += intersection_cond_1d(a1,b2,b3,plane_parameters.at(i).at(5),plane_parameters.at(i).at(6));
                
               count += intersection_cond_1d(a2,b2,b3,plane_parameters.at(i).at(7),plane_parameters.at(i).at(6));
                   
               count += intersection_cond_1d(a3,b2,b3,plane_parameters.at(i).at(2),plane_parameters.at(i).at(6));
                   
               if( count==3 ) //then link 3 intersects with obstacle
               break;
               
               else{
                count=0;
                
                count += intersection_cond_1d(a1,b1,b2,plane_parameters.at(i).at(5),plane_parameters.at(i).at(6));
                
                count += intersection_cond_1d(a2,b1,b2,plane_parameters.at(i).at(7),plane_parameters.at(i).at(6));
                   
                count += intersection_cond_1d(a3,b1,b2,plane_parameters.at(i).at(2),plane_parameters.at(i).at(6));
                
                if( count==3 ) //then link 2 intersects with obstacle
                 break;
                else
                    count_ok++;   //no of obstacles not in collision
               }
               
//               if(count_ok!=0){
//                   break;
//               }
           }
           if( count_ok == plane_parameters.size() )
               return 0;   // no intersection
           else
               return 1;  //indicates intersection
       }
     }
     
     if( a == 2 ){
        if( (link4_link6_intersection() == 1) ){
           //cout<<"\n\nLink 1 and link 3 intersect here: \n";
           //display_state();//-------------------------------------------------display_state
           return 1;                  //indicates intersection
       }
       
     else{
           //check for each obstalce
           int count_ok{};
           for( int i=0; i<plane_parameters.size() ; i++ ){   //obstacle vector and not plne parameters
               int count = 0;
               vector <float> a1 { (plane_parameters.at(i).at(5).at(0) - plane_parameters.at(i).at(6).at(0)) , (plane_parameters.at(i).at(5).at(1) - plane_parameters.at(i).at(6).at(1)) , (plane_parameters.at(i).at(5).at(2) - plane_parameters.at(i).at(6).at(2)) }; 
               vector <float> a2 { (plane_parameters.at(i).at(7).at(0) - plane_parameters.at(i).at(6).at(0)) , (plane_parameters.at(i).at(7).at(1) - plane_parameters.at(i).at(6).at(1)) , (plane_parameters.at(i).at(7).at(2) - plane_parameters.at(i).at(6).at(2)) };
               vector <float> a3 { (plane_parameters.at(i).at(2).at(0) - plane_parameters.at(i).at(6).at(0)) , (plane_parameters.at(i).at(2).at(1) - plane_parameters.at(i).at(6).at(1)) , (plane_parameters.at(i).at(2).at(2) - plane_parameters.at(i).at(6).at(2)) };
               vector <float> b3 { x6 - plane_parameters.at(i).at(6).at(0), y6 - plane_parameters.at(i).at(6).at(1) , z6 - plane_parameters.at(i).at(6).at(2) };
               vector <float> b2 { x5 - plane_parameters.at(i).at(6).at(0), y5 - plane_parameters.at(i).at(6).at(1) , z5 - plane_parameters.at(i).at(6).at(2) };
               vector <float> b1 { x4 - plane_parameters.at(i).at(6).at(0), y4 - plane_parameters.at(i).at(6).at(1) , z4 - plane_parameters.at(i).at(6).at(2) };
               
               count += intersection_cond_1d(a1,b2,b3,plane_parameters.at(i).at(5),plane_parameters.at(i).at(6));
                
               count += intersection_cond_1d(a2,b2,b3,plane_parameters.at(i).at(7),plane_parameters.at(i).at(6));
                   
               count += intersection_cond_1d(a3,b2,b3,plane_parameters.at(i).at(2),plane_parameters.at(i).at(6));
                   
               if( count==3 ) //then link 3 intersects with obstacle
               break;
               
               else{
                count=0;
                
                count += intersection_cond_1d(a1,b1,b2,plane_parameters.at(i).at(5),plane_parameters.at(i).at(6));
                
                count += intersection_cond_1d(a2,b1,b2,plane_parameters.at(i).at(7),plane_parameters.at(i).at(6));
                   
                count += intersection_cond_1d(a3,b1,b2,plane_parameters.at(i).at(2),plane_parameters.at(i).at(6));
                
                if( count==3 ) //then link 2 intersects with obstacle
                 break;
                else
                    count_ok++;   //no of obstacles not in collision
               }
               
//               if(count_ok!=0){
//                   break;
//               }
           }
           if( count_ok == plane_parameters.size() )
               return 0;   // no intersection
           else
               return 1;  //indicates intersection
       }
       
     }
     
     if( a == 3 ){
        if( (link7_link9_intersection() == 1) ){
           //cout<<"\n\nLink 1 and link 3 intersect here: \n";
           //display_state();//-------------------------------------------------display_state
           return 1;                  //indicates intersection
       }
       
     else{
           //check for each obstalce
           int count_ok{};
           for( int i=0; i<plane_parameters.size() ; i++ ){   //obstacle vector and not plne parameters
               int count = 0;
               vector <float> a1 { (plane_parameters.at(i).at(5).at(0) - plane_parameters.at(i).at(6).at(0)) , (plane_parameters.at(i).at(5).at(1) - plane_parameters.at(i).at(6).at(1)) , (plane_parameters.at(i).at(5).at(2) - plane_parameters.at(i).at(6).at(2)) }; 
               vector <float> a2 { (plane_parameters.at(i).at(7).at(0) - plane_parameters.at(i).at(6).at(0)) , (plane_parameters.at(i).at(7).at(1) - plane_parameters.at(i).at(6).at(1)) , (plane_parameters.at(i).at(7).at(2) - plane_parameters.at(i).at(6).at(2)) };
               vector <float> a3 { (plane_parameters.at(i).at(2).at(0) - plane_parameters.at(i).at(6).at(0)) , (plane_parameters.at(i).at(2).at(1) - plane_parameters.at(i).at(6).at(1)) , (plane_parameters.at(i).at(2).at(2) - plane_parameters.at(i).at(6).at(2)) };
               vector <float> b3 { x9 - plane_parameters.at(i).at(6).at(0), y9 - plane_parameters.at(i).at(6).at(1) , z9 - plane_parameters.at(i).at(6).at(2) };
               vector <float> b2 { x8 - plane_parameters.at(i).at(6).at(0), y8 - plane_parameters.at(i).at(6).at(1) , z8 - plane_parameters.at(i).at(6).at(2) };
               vector <float> b1 { x7 - plane_parameters.at(i).at(6).at(0), y7 - plane_parameters.at(i).at(6).at(1) , z7 - plane_parameters.at(i).at(6).at(2) };
               
               count += intersection_cond_1d(a1,b2,b3,plane_parameters.at(i).at(5),plane_parameters.at(i).at(6));
                
               count += intersection_cond_1d(a2,b2,b3,plane_parameters.at(i).at(7),plane_parameters.at(i).at(6));
                   
               count += intersection_cond_1d(a3,b2,b3,plane_parameters.at(i).at(2),plane_parameters.at(i).at(6));
                   
               if( count==3 ) //then link 3 intersects with obstacle
               break;
               
               else{
                count=0;
                
                count += intersection_cond_1d(a1,b1,b2,plane_parameters.at(i).at(5),plane_parameters.at(i).at(6));
                
                count += intersection_cond_1d(a2,b1,b2,plane_parameters.at(i).at(7),plane_parameters.at(i).at(6));
                   
                count += intersection_cond_1d(a3,b1,b2,plane_parameters.at(i).at(2),plane_parameters.at(i).at(6));
                
                if( count==3 ) //then link 2 intersects with obstacle
                 break;
                else
                    count_ok++;   //no of obstacles not in collision
               }
               
//               if(count_ok!=0){
//                   break;
//               }
           }
           if( count_ok == plane_parameters.size() )
               return 0;   // no intersection
           else
               return 1;  //indicates intersection
       }
       
     }
     
    }
    
    int link1_link3_intersection(){
     float alpha1{10000000000.0}, alpha2{10000000000.0};
        if(x3 != x2){  //else alpha1 can take any value-------------------------------------------------------------------------------------------------------------------
            alpha1 = (-x2) / (x3-x2);
        }
        
        if(z3 != z2){  //else alpha2 can take any value--------------------------------------------------------------------------------------------------------------------
            alpha2 = (-z2) / (z3-z2);
        }
        
        if( (alpha1 == alpha2) && (alpha1 > 0) && (alpha1 < 1) ){
            if( ( (y2 + alpha1*(y3-y2)) > 0 ) && ( (y2 + alpha1*(y3-y2)) < l1 ) )
                return 1;                                   //indicates intersection
            else
                return 0;
        }
        else
            return 0;
    }
    
    int link4_link6_intersection(){
     float alpha1{10000000000.0}, alpha2{10000000000.0};
        if( x6 != x5 ){  //else alpha1 can take any value-------------------------------------------------------------------------------------------------------------------
            alpha1 = ( - x5 +xn2 ) / ( x6 - x5 );
        }
        
        if( z6 != z5 ){  //else alpha2 can take any value--------------------------------------------------------------------------------------------------------------------
            alpha2 = ( - z5 +zn2 ) / ( z6 - z5 );
        }
        
        if( (alpha1 == alpha2) && (alpha1 > 0) && (alpha1 < 1) ){
            if( ( (y5 + alpha1*(y6-y5)) > 0 ) && ( (y5 + alpha1*(y6-y2)) < l1 ) )
                return 1;                                   //indicates intersection
            else
                return 0;
        }
        else
            return 0;
    }
    
    int link7_link9_intersection(){
     float alpha1{10000000000.0}, alpha2{10000000000.0};
        if( x9 != x8 ){  //else alpha1 can take any value-------------------------------------------------------------------------------------------------------------------
            alpha1 = ( - x8 +xn3 ) / ( x9 - x8 );
        }
        
        if( z9 != z8 ){  //else alpha2 can take any value--------------------------------------------------------------------------------------------------------------------
            alpha2 = ( - z8 +zn3 ) / ( z9 - z8 );
        }
        
        if( (alpha1 == alpha2) && (alpha1 > 0) && (alpha1 < 1) ){
            if( ( (y5 + alpha1*(y9-y8)) > 0 ) && ( (y8 + alpha1*(y9-y8)) < l1 ) )
                return 1;                                   //indicates intersection
            else
                return 0;
        }
        else
            return 0;
    }
    
     float euclid_dist_to_state_in_cspace(const state &s){
        return ( sqrt( (theta1-s.theta1)*(theta1-s.theta1) + (theta2-s.theta2)*(theta2-s.theta2) + (theta3-s.theta3)*(theta3-s.theta3) ) );
    }

    float euclid_dist_to_goal_in_cspace(){
        return ( euclid_dist( vector<float>{x3,y3,z3} , vector<float>{xe1,ye1,ze1} ) + euclid_dist( vector<float>{x6,y6,z6} , vector<float>{xe2,ye2,ze2} ) +
                euclid_dist( vector<float>{x9,y9,z9} , vector<float>{xe3,ye3,ze3} ) );
    }

/*c*/int check_line_intersection(){ // (0-state invalid)  (1-state valid)
        vector < vector <float>> p { {0,0},{x1,y1},{x2,y2},{x3,y3} };
        vector < vector <float>> q { {xn,yn},{x4,y4},{x5,y5},{x6,y6} };
        
        int count{};
        if( (do_intersect( q.at(2), q.at(3), p.at(2), p.at(3) ) == 1) || 
           (do_intersect( q.at(2), q.at(3), p.at(1), p.at(2) ) == 1) ||
           (do_intersect( q.at(2), q.at(3), p.at(0), p.at(1) ) == 1) ||
           (do_intersect( q.at(1), q.at(2), p.at(2), p.at(3) ) == 1) ||
           (do_intersect( q.at(1), q.at(2), p.at(1), p.at(2) ) == 1) ||
           (do_intersect( q.at(1), q.at(2), p.at(0), p.at(1) ) == 1) ||
           (do_intersect( q.at(0), q.at(1), p.at(2), p.at(3) ) == 1) ||
           (do_intersect( q.at(0), q.at(1), p.at(1), p.at(2) ) == 1) )
            count++;
        
        if( count == 1 ){
            return 0;
        }
        
        else
            return 1;
            
    }
    
    int do_intersect( const vector <float> &p1, const vector <float> &q1, const vector <float> &p2, const vector <float> &q2 ){
        int o1 = orientation(p1, q1, p2);
        int o2 = orientation(p1, q1, q2);
        int o3 = orientation(p2, q2, p1);
        int o4 = orientation(p2, q2, q1);
        
        if (o1 != o2 && o3 != o4)
        return true;
 
    // Special Cases
    // p1, q1 and p2 are collinear and p2 lies on segment p1q1
        if (o1 == 0 && onSegment(p1, p2, q1)) return true;
 
    // p1, q1 and q2 are collinear and q2 lies on segment p1q1
        if (o2 == 0 && onSegment(p1, q2, q1)) return true;
 
    // p2, q2 and p1 are collinear and p1 lies on segment p2q2
        if (o3 == 0 && onSegment(p2, p1, q2)) return true;
 
     // p2, q2 and q1 are collinear and q1 lies on segment p2q2
        if (o4 == 0 && onSegment(p2, q1, q2)) return true;
 
    return false;
    }
    
    int orientation( const vector <float> &p , const vector <float> &q , const vector <float> &r ){
    float val = (q.at(1) - p.at(1)) * (r.at(0) - q.at(0) ) -
              (q.at(0) - p.at(0) ) * (r.at(1)- q.at(1));
 
    if (val == 0) return 0;  // collinear
 
    return (val > 0)? 1: 2;
    }
    
    bool onSegment( const vector<float> p, const vector<float> q, const vector<float> r)
    {
    if (q.at(0)<= max(p.at(0), r.at(0)) && q.at(0)>= min(p.at(0), r.at(0)) &&
        q.at(1) <= max(p.at(1), r.at(1) ) && q.at(1) >= min(p.at(1), r.at(1) ))
       return true;
 
    return false;
    }
    
    
    ~state() {}
};

//struct addstate : state {
//    int state_number;
//    int previous_state;
//    addstate(int sn, int ps){
//        state_number=sn;
//        previous_state=ps;
//    }
//};

vector <vector <vector <float>>> obstacle_vector{};
vector <state> graph {};


//function definitions
float theta_vector(float p1x, float p1y, float p2x, float p2y);  //-----> calculate angle (theta) value
void generate_c_cpace( vector <vector <vector <int>>> &c_space , vector <vector <vector <int>>> &c_space_man2 , vector <vector <vector <int>>> &c_space_man2 );  //----------------------------------------> generates c space for the manipulator
vector <float> get_plane_parameters(int p, int q, int r, int i);
float dist_states(const state &a, const state&s);
state getnearstate(const state &s, const vector <state> &graph);
vector <vector <float>> generate_input (const state &near_state);
state find_new_state(const vector <vector <float>> &urand, const state &near_state, const state &rand_state);
int ifisstatevalid( state &s,const vector <vector <vector <int>>> &c_space, const vector <vector <vector <int>>> &c_space_man2);
int check_state_collision(const state &s,const vector <vector <vector <int>>> &c_space, const vector <vector <vector <int>>> &c_space_man2);
state runge_kutta( const state &near_state, const vector <float> &urandom);



int main(){
//    obstacle_vector={ {{5,0,5},{5,0,-5},{-5,0,-5},{-5,0,5},{5,-5,5},{5,-5,-5},{-5,-5,-5},{-5,-5,5}} };
//    obstacle_vector={ {{5,8,5},{5,8,-5},{-5,8,-5},{-5,8,5},{5,5,5},{5,5,-5},{-5,5,-5},{-5,5,5}} };
    obstacle_vector={ {{5,0.5,5},{5,0.5,-5},{-5,0.5,-5},{-5,0.5,5},{5,-5,5},{5,-5,-5},{-5,-5,-5},{-5,-5,5}} };
    
    //cout << "Starts now" << endl;
    
    vector <float> temp_thetas{};
    vector <float> temp_thetas_2{};
    vector <float> temp_thetas_3{};
    cout << "\nStart point "<< start_point_man1.at(0) <<"\t"<< start_point_man1.at(1) <<"\t"<< start_point_man1.at(2) <<"\n";
    cout << "\nStart point "<< start_point_man2.at(0) <<"\t"<< start_point_man2.at(1) <<"\t"<< start_point_man2.at(2) <<"\n";
    cout << "\nStart point "<< start_point_man3.at(0) <<"\t"<< start_point_man3.at(1) <<"\t"<< start_point_man3.at(2) <<"\n";
    temp_thetas = get_thetas(start_point_man1,1);
    temp_thetas_2 = get_thetas(start_point_man2,2);
    temp_thetas_3 = get_thetas(start_point_man3,3);
    state initial_state(temp_thetas.at(0), temp_thetas.at(1), temp_thetas.at(2), temp_thetas_2.at(0), temp_thetas_2.at(1), temp_thetas_2.at(2), temp_thetas_3.at(0), temp_thetas_3.at(1), temp_thetas_3.at(2) );
    cout << "\nEnd point"<< end_point_man1.at(0) <<"\t"<< end_point_man1.at(1) << "\t" << end_point_man1.at(2) <<"\n";
    temp_thetas = get_thetas(end_point_man1,1);
    temp_thetas_2 = get_thetas(end_point_man2,2);
    temp_thetas_3 = get_thetas(end_point_man3,3);
    state final_state(temp_thetas.at(0), temp_thetas.at(1), temp_thetas.at(2), temp_thetas_2.at(0), temp_thetas_2.at(1), temp_thetas_2.at(2), temp_thetas_3.at(0), temp_thetas_3.at(1), temp_thetas_3.at(2) );
    
    cout<<"\nThe initial_state of manipulators is: \n";
    initial_state.display_state();
    cout<<"\nThe final state of manipulators is: \n";
    final_state.display_state();
    
//    cout<<"\nThe initial_states of manipulators are: \n";
//    initial_state.display_state();
    
    //generate c-space
    vector <vector <vector <int>>> c_space;
    vector <vector <vector <int>>> c_space_man2;
    vector <vector <vector <int>>> c_space_man3;
    generate_c_cpace(c_space,c_space_man2,c_space_man3);
    cout<<"\n\n\n*********   C SPACE GENERATED   *******************\n\n\n";
    
    ofstream myfile;
    myfile.open ("P3_Cspace_man1.csv");
    for(int i=0; i<c_space.size(); i++){
        for(int j=0; j<c_space.at(i).size(); j++){
            for(int k=0; k<c_space.at(i).at(j).size(); k++){
                if( c_space.at(i).at(j).at(k)==1 )
                   myfile<<i<<","<<j<<","<<k<<endl;
                //myfile<<endl;
            }
        }
        //myfile<<endl;
    }
    myfile.close();
    
    myfile.open ("P3_Cspace_man2.csv");
    for(int i=0; i<c_space_man2.size(); i++){
        for(int j=0; j<c_space_man2.at(i).size(); j++){
            for(int k=0; k<c_space_man2.at(i).at(j).size(); k++){
                if( c_space_man2.at(i).at(j).at(k)==1 )
                   myfile<<i<<","<<j<<","<<k<<endl;
                //myfile<<endl;
            }
        }
        //myfile<<endl;
    }
    myfile.close();
    
    myfile.open ("P3_Cspace_man3.csv");
    for(int i=0; i<c_space_man3.size(); i++){
        for(int j=0; j<c_space_man3.at(i).size(); j++){
            for(int k=0; k<c_space_man3.at(i).at(j).size(); k++){
                if( c_space_man3.at(i).at(j).at(k)==1 )
                   myfile<<i<<","<<j<<","<<k<<endl;
                //myfile<<endl;
            }
        }
        //myfile<<endl;
    }
    myfile.close();
    
    cout<<"\nThe C-Space is exported!!!!!!!!!!!!!!!\n";
    
    //------>> for displaying the c space on the command prompt
//    for(int i=0; i<c_space.size(); i++){ cout<<"\n\nFor angle theta1= "<<i<<endl;
//        for(int j=0; j<c_space.at(i).size(); j++){
//            for(int k=0; k<c_space.at(i).at(j).size(); k++){
//                   cout<<c_space.at(i).at(j).at(k);
//                //myfile<<endl;
//            }cout<<endl;
//        }
//        //myfile<<endl;
//    }
    
    srand(time(0));
    
    state newstate;
    state rand_state;
    int count=0, point_number=0;
    float least_distance_ever = INT_MAX;
    
    initial_state.state_number=1;
    initial_state.previous_state=0;
    graph.push_back(initial_state);
    
    vector <int> manip_goal{0,0,0};
    while( ( newstate.isstate_goal()==0 ) && ( count <= 50000 ) ){
        
        //if any of the manipulators reach goal, they have to stay there
          //create func for goal checking for each manip in the struct
        if ( newstate.ismanip_1_goal() == 1 ){
            if( manip_goal.at(0)=0 ){
                cout<<"\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n**********The 1st manipulator has reached the goal!!\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n**********";
            }
            manip_goal.at(0)=1;
        }
            
        if ( newstate.ismanip_2_goal() == 1 ){
            if( manip_goal.at(1)=0 ){
                cout<<"\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n**********The 2nd manipulator has reached the goal!!\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n**********";
            }
            manip_goal.at(1)=1;
        }
            
        if ( newstate.ismanip_3_goal() == 1 ){
            if( manip_goal.at(2)=0 ){
                cout<<"\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n**********The 3rd manipulator has reached the goal!!\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n**********";
            }
            manip_goal.at(2)=1;
        }
        
        rand_state.randomize_state( vector <float> {final_state.theta1,final_state.theta2,final_state.theta3}, 
                                    vector <float> {final_state.theta4,final_state.theta5,final_state.theta6},
                                    vector <float> {final_state.theta7,final_state.theta8,final_state.theta9},
                                    vector <int> manip_goal);
//        cout<<"\nThis is the random state created: \n";
//        rand_state.display_state();
        
        state near_state = getnearstate(rand_state,graph);
//        cout<<"\nThis is the near state found: \n";
//        near_state.display_state();
        
        vector < vector <float> > urand = generate_input( near_state );
        newstate = find_new_state(urand,near_state,rand_state);
        newstate.time = near_state.time + 1; //--------------------->> actuator duration
//        cout<<"\nThis is the new state found: \n";
//        newstate.display_state();       
        //state validity function - ifisstatevalid
        //--------------->> can implement path validity as well
        
        if( ifisstatevalid( newstate, c_space , c_space_man2 ) == 1){
            state temp_state{};
            temp_state = newstate;
            temp_state.state_number = (point_number+2) ;
            temp_state.previous_state = near_state.state_number;
//          temp_state.time = near_state.time + 1; //--------------------->> actuator duration
            graph.push_back(temp_state);
            cout<<"\nYes, the state is valid\n";
            point_number++;
        }
        
        else
         {cout<<"\nThe state is invalid\n";continue;}
         count++;
         cout<<"\nNo of iterations over: "<<count<<endl;
         
         if( newstate.isstate_goal()==1 ){
             cout<<"\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n**********GOAL REACHED\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n**********";
             newstate.display_state();
         }
         
         if( least_distance_ever > newstate.euclid_dist_to_goal_in_cspace() ){
             least_distance_ever = newstate.euclid_dist_to_goal_in_cspace();
         }
         
         if( count%10 == 0 ){
             cout<<"\n\n\n\n\n\n\n\nLeast_distance_ever is: "<<least_distance_ever<<endl<<endl<<endl;
         }
    }
    
    cout<<"\n\nDisplaying graph points:\n";
    for(int i=0; i<graph.size(); ++i ){
        graph.at(i).display_state();
//        cout<<graph.at(i).theta1<<","<<graph.at(i).theta2<<","<<graph.at(i).theta3<<" sn: "<<graph.at(i).state_number<<" ps: "<<graph.at(i).previous_state<<endl;
    }
    
    myfile.open ("Final_project_c_graph_points.csv");
    for(int i=0; i<graph.size(); i++){
        myfile<<graph.at(i).theta1<<","<<graph.at(i).theta2<<","<<graph.at(i).theta3<<","<<graph.at(i).theta4<<","<<graph.at(i).theta5<<","<<graph.at(i).theta6<<","<<graph.at(i).theta7<<","<<graph.at(i).theta8<<","<<graph.at(i).theta9;
        myfile<<endl;
    }
    myfile.close();
    
    vector <state> state_record{};
    int current_point=graph.at(graph.size()-1).previous_state;
    state_record.push_back(graph.at(graph.size()-1));
    
    while(current_point!=0){
       for(int i=0; i<graph.size(); i++){
           if(current_point==graph.at(i).state_number){
             state_record.push_back( graph.at(i) );
             current_point=graph.at(i).previous_state;
             break;
           }
       } 
    }
    
    cout<<"\n\nDisplaying reverse path for manipulator 1:\n";
    for(int i=0; i<state_record.size(); ++i ){
        cout<<state_record.at(i).theta1<<","<<state_record.at(i).theta2<<","<<state_record.at(i).theta3<<endl;
    }
    
    cout<<"\n\nDisplaying reverse path for manipulator 2:\n";
    for(int i=0; i<state_record.size(); ++i ){
        cout<<state_record.at(i).theta4<<","<<state_record.at(i).theta5<<","<<state_record.at(i).theta6<<endl;
    }
    
    cout<<"\n\nDisplaying reverse path for manipulator 3:\n";
    for(int i=0; i<state_record.size(); ++i ){
        cout<<state_record.at(i).theta7<<","<<state_record.at(i).theta8<<","<<state_record.at(i).theta9<<endl;
    }
    
    myfile.open ("Final_project_c_show_path.csv");
    for(int i=0; i<state_record.size(); i++){
        myfile<<state_record.at(i).theta1<<","<<state_record.at(i).theta2<<","<<state_record.at(i).theta3<<","<<state_record.at(i).theta4<<","<<state_record.at(i).theta5<<","<<state_record.at(i).theta6<<","<<graph.at(i).theta7<<","<<graph.at(i).theta8<<","<<graph.at(i).theta9;
        myfile<<endl;
    }
    myfile.close();
    
    return 0;
}

float dist_states(const state &a, const state&s){
    float dist{};
    dist += ( (a.theta1 - s.theta1)*(a.theta1 - s.theta1) + 
                (a.theta2 - s.theta2)*(a.theta2 - s.theta2) + 
                (a.theta3 - s.theta3)*(a.theta3 - s.theta3) +
                (a.theta4 - s.theta4)*(a.theta4 - s.theta4) +
                (a.theta5 - s.theta5)*(a.theta5 - s.theta5) + 
                (a.theta6 - s.theta6)*(a.theta6 - s.theta6)
                (a.theta7 - s.theta7)*(a.theta7 - s.theta7) +
                (a.theta8 - s.theta8)*(a.theta8 - s.theta8) + 
                (a.theta9 - s.theta9)*(a.theta9 - s.theta9) );
    dist = sqrt(dist);
    return ( dist );
}

state getnearstate(const state &s, const vector <state> &graph){
    float least_distance=INT_MAX;
    state nearest_state(0,0,0,0,0,0,0,0,0);
    for(int i=0; i<graph.size(); ++i){
       float dist = dist_states( graph.at(i) ,s );
    
     if( least_distance>dist ){
        least_distance=dist;
        nearest_state=graph.at(i);
     }    
    }
    return nearest_state;
}

vector <vector <float>> generate_input (const state &near_state){  //function to get random input speeds
    vector < vector <float> > urand{};
    float theta_dot_max = 0.1;               // actuator speed is +-0.1 rad/s or 5 deg/s
    float theta_dot_min = -0.1;
    for(int i=0; i<20; i++){
        vector <float> u{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
        
        for(int j=0; j<u.size(); j++)
            u.at(j) = rand() % static_cast<int>( theta_dot_max*1000 +1 - theta_dot_min*1000 ) + static_cast<int>(theta_dot_min*1000); u.at(j)/=1000;

        urand.push_back(u);
    }
    return urand;
}

state find_new_state(const vector <vector <float>> &urand, const state &near_state, const state &rand_state){
    state new_state;
    state this_state;
    
    float least_distance = INT_MAX;
    for(int i=0; i<urand.size(); ++i){
     this_state = runge_kutta( near_state, urand.at(i) );
     if( least_distance > dist_states(this_state,rand_state) ){
         least_distance = dist_states(this_state,rand_state);
        new_state = this_state;
     }
    }
    
    return new_state; 
}

state runge_kutta( const state &near_state, const vector <float> &urandom){
    float time = 1.0;    //-----------------------------------------------------------------------------> time for each actuation (can reduce this)
    
    state this_state ( (near_state.theta1 + time * urandom.at(0)), (near_state.theta2 + time * urandom.at(1)), (near_state.theta3 + time * urandom.at(2)),
                        (near_state.theta4 + time * urandom.at(3)), (near_state.theta5 + time * urandom.at(4)), (near_state.theta6 + time * urandom.at(5)),
                        (near_state.theta7 + time * urandom.at(7)), (near_state.theta8 + time * urandom.at(7)), (near_state.theta9 + time * urandom.at(8)));
    return this_state;
}

int ifisstatevalid( state &s, const vector <vector <vector <int>>> &c_space, const vector <vector <vector <int>>> &c_space_man2){  // (0-state invalid)  (1-state valid)

    if ( (s.theta1>=0) && (s.theta1<=(2*pi)) &&
        (s.theta2>=0) && (s.theta2<=(2*pi)) &&
        (s.theta3>=0) && (s.theta3<=(2*pi)) &&
        (s.theta4>=0) && (s.theta4<=(2*pi)) &&
        (s.theta5>=0) && (s.theta5<=(2*pi)) &&
        (s.theta6>=0) && (s.theta6<=(2*pi)) &&
        (s.theta7>=0) && (s.theta7<=(2*pi)) &&
        (s.theta8>=0) && (s.theta8<=(2*pi)) &&
        (s.theta9>=0) && (s.theta9<=(2*pi)) ){ //--------------->> can implement path validity as well
          cout<<"\nThe thetas are in the range!!"<<s.theta1<<" "<<s.theta2<<" "<<s.theta3<<" "<<s.theta4<<" "<<s.theta5<<" "<<s.theta6<<" "<<s.theta7<<" "<<s.theta8<<" "<<s.theta9<<"\n";
          if(check_state_collision(s, c_space, c_space_man2) == 0)
            return 0;
          else{
              if( (s.theta1>6.1784) && (s.theta1<0.1047) && (s.theta4>3.037) && (s.theta4<3.2463) ){
               if( s.check_line_intersection() == 0 )            //check line intersection
                return 0;
               else 
                   return 1;
              }
             return 1; 
          }
        }
    else
        return 0;
}
  //static_cast
int check_state_collision(const state &s,const vector <vector <vector <int>>> &c_space, const vector <vector <vector <int>>> &c_space_man2){  //----------------------->> has a safety of 2 points from the c space

    cout<<"\nIn check_state_collision function...\n";
    
    float theta1 = s.theta1 * (180/pi);
    float theta2 = s.theta2 * (180/pi);
    float theta3 = s.theta3 * (180/pi);
    float theta4 = s.theta4 * (180/pi);
    float theta5 = s.theta5 * (180/pi);
    float theta6 = s.theta6 * (180/pi);
    float theta7 = s.theta7 * (180/pi);
    float theta8 = s.theta8 * (180/pi);
    float theta9 = s.theta9 * (180/pi);
    
    int a = static_cast<int>( ( theta1 - ( fmod(theta1,2) ) ) / 2);
    cout<<"\nAngle number a : "<<a<<endl;
    int b = static_cast<int>( ( theta2 - ( fmod(theta2,2) ) ) / 2);
    cout<<"\nAngle number b : "<<b<<endl;
    int c = static_cast<int>( ( theta3 - ( fmod(theta3,2) ) ) / 2);
    cout<<"\nAngle number c : "<<c<<endl;
    int d = static_cast<int>( ( theta4 - ( fmod(theta4,2) ) ) / 2);
    cout<<"\nAngle number d : "<<d<<endl;
    int e = static_cast<int>( ( theta5 - ( fmod(theta5,2) ) ) / 2);
    cout<<"\nAngle number e : "<<e<<endl;
    int f = static_cast<int>( ( theta6 - ( fmod(theta6,2) ) ) / 2);
    cout<<"\nAngle number f : "<<f<<endl;
    int g = static_cast<int>( ( theta7 - ( fmod(theta7,2) ) ) / 2);
    cout<<"\nAngle number g : "<<d<<endl;
    int h = static_cast<int>( ( theta8 - ( fmod(theta8,2) ) ) / 2);
    cout<<"\nAngle number h : "<<e<<endl;
    int i1 = static_cast<int>( ( theta9 - ( fmod(theta9,2) ) ) / 2);
    cout<<"\nAngle number i1 : "<<f<<endl;
    
    vector <int> thetas1 { (a) , (a-1) , (a+1) , (a+2) };
    vector <int> thetas2 { (b) , (b-1) , (b+1) , (b+2) };
    vector <int> thetas3 { (c) , (c-1) , (c+1) , (c+2) };
    vector <int> thetas4 { (d) , (d-1) , (d+1) , (d+2) };
    vector <int> thetas5 { (e) , (e-1) , (e+1) , (e+2) };
    vector <int> thetas6 { (f) , (f-1) , (f+1) , (f+2) };
    vector <int> thetas7 { (g) , (g-1) , (g+1) , (g+2) };
    vector <int> thetas8 { (h) , (h-1) , (h+1) , (h+2) };
    vector <int> thetas9 { (i1) , (i1-1) , (i1+1) , (i1+2) };
    
    if(a<1)
        thetas1.at(1) = a;
        
    if(b<1)
        thetas2.at(1) = b;
        
    if(c<1)
        thetas3.at(1) = c;
        
    if(d<1)
        thetas4.at(1) = d;
        
    if(e<1)
        thetas5.at(1) = e;
        
    if(f<1)
        thetas6.at(1) = f;
        
    if(g<1)
        thetas7.at(1) = g;
        
    if(h<1)
        thetas8.at(1) = h;
        
    if(i1<1)
        thetas9.at(1) = i1;
    
    int count{};
    for( int i=0; i<thetas1.size(); i++ ){
        for( int j=0; j<thetas2.size(); j++ ){
            for( int k=0; k<thetas3.size(); k++ ){
                if( c_space.at( thetas1.at(i) ).at( thetas2.at(j) ).at( thetas3.at(k) ) == 1 )
                    count++;
                if(count!=0)
                    break;
            }
            if(count!=0)
                break;
        }
        if(count!=0)
            break;
    }
    
    for( int i=0; i<thetas4.size(); i++ ){
        for( int j=0; j<thetas5.size(); j++ ){
            for( int k=0; k<thetas6.size(); k++ ){
                if( c_space_man2.at( thetas4.at(i) ).at( thetas5.at(j) ).at( thetas6.at(k) ) == 1 )
                    count++;
                if(count!=0)
                    break;
            }
            if(count!=0)
                break;
        }
        if(count!=0)
            break;
    }
    
    for( int i=0; i<thetas7.size(); i++ ){
        for( int j=0; j<thetas8.size(); j++ ){
            for( int k=0; k<thetas9.size(); k++ ){
                if( c_space_man2.at( thetas7.at(i) ).at( thetas8.at(j) ).at( thetas9.at(k) ) == 1 )
                    count++;
                if(count!=0)
                    break;
            }
            if(count!=0)
                break;
        }
        if(count!=0)
            break;
    }
    
    if( count != 0 )//state not valid
        {cout<<"\nThe state is not valid!"<<endl;return 0;}
    else{
        cout<<"\nYes, state is valid!!!\n";
        return 1;
    }
    
}

void generate_c_cpace( vector <vector <vector <int>>> &c_space , vector <vector <vector <int>>> &c_space_man2 , vector <vector <vector <int>>> &c_space_man3 ){
    vector <vector <vector <float>>> plane_parameters {};
//  vector <vector <vector <int>>> c_space_obstacles (180, vector <vector <int>>(180, vector <int> (180) ) );
    vector <vector <vector <int>>> c_space_obstacles_man1 {};
    vector <vector <vector <int>>> c_space_obstacles_man2 {};
    vector <vector <vector <int>>> c_space_obstacles_man3 {};
    cout<<"\n\nGenerating C-space:\n\n";
    
    //get plane equations for all the obstalces (cubes, cuboids)
    for(int i=0 ; i<obstacle_vector.size() ; i++){
        vector <vector <float>> temp_vec{};
        temp_vec.push_back( get_plane_parameters(1,2,3,i) );
        temp_vec.push_back( get_plane_parameters(5,6,7,i) );
        temp_vec.push_back( get_plane_parameters(5,6,2,i) );
        temp_vec.push_back( get_plane_parameters(8,7,3,i) );
        temp_vec.push_back( get_plane_parameters(5,1,4,i) );
        temp_vec.push_back( get_plane_parameters(6,2,3,i) );
        plane_parameters.push_back(temp_vec);
    }
    
    cout<<"\n\nThe plane parameters are:\n";
    for(int i=0 ; i<plane_parameters.size() ; i++){
        for(int j=0 ; j<plane_parameters.at(i).size() ; j++){
            for(int k=0 ; k<plane_parameters.at(i).at(j).size() ; k++)
                cout<<plane_parameters.at(i).at(j).at(k)<<" ";
            cout<<endl;
        }
        cout<<endl;
    }
    cout<<endl<<endl;
    
    cout<<"\nThe c space for the three manipulators simultaneously:\n\n";
    for(int i=0 ; i<360 ; i+=2){   //-------> for theta1
        vector <vector <int>> temp2_vector_man1{};
        vector <vector <int>> temp2_vector_man2{};
        vector <vector <int>> temp2_vector_man3{};
        for(int j=0 ; j<360 ; j+=2){    //-------> for theta2
            
            vector <int> temp_vector_man1{};
            vector <int> temp_vector_man2{};
            vector <int> temp_vector_man3{};
            for(int k=0 ; k<360 ; k+=2){   //-------> for theta3
                
                //check collision for all angles tomorrow
                //must take 2 hrs, but do it fast
                state check_this_state_man1 ( (i*pi/180), (j*pi/180), (k*pi/180), 0, 0, 0, 0, 0, 0 );
                state check_this_state_man2 ( 0, 0, 0, (i*pi/180), (j*pi/180), (k*pi/180), 0, 0, 0 );
                state check_this_state_man3 ( 0, 0, 0, 0, 0, 0, (i*pi/180), (j*pi/180), (k*pi/180) );
                if( check_this_state_man1.check_state(obstacle_vector,1) == 1 ){
                   temp_vector_man1.push_back(1);
                   //c_space_obstacles.at(i/2).at(j/2).at(k/2) = 1;   //intersection happens
                   //check_this_state.display_state();//-------------------------------------------------display_state
                }
                else
                    temp_vector_man1.push_back(0);
                    //c_space_obstacles.at(i).at(j).at(k) = 0;
                 
                if( check_this_state_man2.check_state(obstacle_vector,2) == 1 ){
                    temp_vector_man2.push_back(1);
                }
                else
                    temp_vector_man2.push_back(0);
                    
                if( check_this_state_man3.check_state(obstacle_vector,3) == 1 ){
                    temp_vector_man3.push_back(1);
                }
                else
                    temp_vector_man3.push_back(0);
                
            }
            
            temp2_vector_man1.push_back(temp_vector_man1);
            temp2_vector_man2.push_back(temp_vector_man2);
            temp2_vector_man3.push_back(temp_vector_man3);
        }
        
        c_space_obstacles_man1.push_back(temp2_vector_man1);
        c_space_obstacles_man2.push_back(temp2_vector_man2);
        c_space_obstacles_man3.push_back(temp2_vector_man3);
        cout<<"Constucted c space for angle:"<<i<<"..."<<endl;
    }
    
    c_space = c_space_obstacles_man1;
    c_space_man2 = c_space_obstacles_man2;
    c_space_man3 = c_space_obstacles_man3;
    
//      return c_space_obstacles;
}

vector <float> get_plane_parameters(int p, int q, int r, int i){   // function which takes 3 point numbers (p,q,r) from obstacle i and spits out the a,b,c,d parameters of plane
    
    float x1 = obstacle_vector.at(i).at(p-1).at(0);
    float y1 = obstacle_vector.at(i).at(p-1).at(1);
    float z1 = obstacle_vector.at(i).at(p-1).at(2);
   
    float x2 = obstacle_vector.at(i).at(q-1).at(0);
    float y2 = obstacle_vector.at(i).at(q-1).at(1);
    float z2 = obstacle_vector.at(i).at(q-1).at(2);

    float x3 = obstacle_vector.at(i).at(r-1).at(0);
    float y3 = obstacle_vector.at(i).at(r-1).at(1);
    float z3 = obstacle_vector.at(i).at(r-1).at(2);
 
    float a = ( (y2-y1)*(z3-z1) - (y3-y1)*(z2-z1) );
    float b = ( -1*(x2-x1)*(z3-z1) + (x3-x1)*(z2-z1) );
    float c = ( (x2-x1)*(y3-y1) - (x3-x1)*(y2-y1) );
    float d = ( -a*x1 - b*y1 - c*z1 );
    
    vector <float> plane_parms {a,b,c,d};
    return (plane_parms);
}

vector <float> get_thetas (const vector <float> &point, const int &manipulator_number) {

     float x3 {};
     float y3 {};
     float z3 {};
    
    if(manipulator_number == 1){
     x3 = point.at(0) - xn1;
     y3 = point.at(1) - yn1;
     z3 = point.at(2) - zn1;
    }
    
    if(manipulator_number == 2){
     x3 = point.at(0) - xn2;
     y3 = point.at(1) - yn2;
     z3 = point.at(2) - zn2; 
    }
    
    if(manipulator_number == 3){
     x3 = point.at(0) - xn3;
     y3 = point.at(1) - yn3;
     z3 = point.at(2) - zn3;
    }
    
    float theta_1 = theta_vector(0,0,x3,z3);
    
    //calculate theta_2
    float x1 = 0;
    float y1 = l1;
    float z1 = 0;
    
    x1 = sqrt(x1*x1 + z1*z1);
    //y1 = y1;
    float x2 = sqrt(x3*x3 + z3*z3);
    float y2 = y3;
    
    float a = ( (l2*l2-l3*l3) - (x1*x1 - x2*x2) - (y1*y1 - y2*y2) ) / (2*y2-2*y1);
    float b = (2*x2 - 2*x1) / (2*y2 - 2*y1);
    float as = ( 1 + b*b ); cout<<"as\t"<<as<<endl;
    float bs = ( -2*x1 - 2*b*(a-y1) );  cout<<"bs\t"<<bs<<endl;
    float cs = ( x1*x1 + (a-y1)*(a-y1) - l2*l2 );  cout<<"cs\t"<<cs<<endl;
    
    float x = (-bs - sqrt( bs*bs - 4*as*cs ) ) / (2*as);
    float y = a - b*x;
    cout << "\nIntersection points: " << x << "\t" << y << endl;
    
    float theta_2 = theta_vector(x1,y1,x,y); cout<<"\nTheta 2 is "<<theta_2<<endl;   //--------------------------------------- not proper, but still works
    if( theta_2 <= (pi/2 + 0.01) )
        theta_2 += (pi/2);
    if( theta_2 >= (3*pi/2-0.001) )
        theta_2 -= 3*pi/2 - 0.001;
//    if ( (0 < theta_2) && (theta_2 < (3*pi/2)) ){
//        theta_2 += pi/2;
//    }
//    //if ( 4.7123 <= theta_2 < 6.28319){
//    else{
//        theta_2 -= (3*pi/2);
//    }
    
    //calculate theta_3
    float theta_3 = theta_vector(x,y,x2,y2) - theta_vector(x1,y1,x,y) + pi;
    if (theta_3 < 0)
        theta_3 += 2*pi;
    
    vector <float> thetas = {theta_1, theta_2, theta_3};
    cout<<"\nThetas: "<<theta_1<<"\t"<<theta_2<<"\t"<<theta_3<<endl;
    return thetas;
}

float theta_vector(float p1x, float p1y, float p2x, float p2y){
    if(((p2y-p1y)>=0)&&((p2x-p1x)>=0))             //1st quadrant
        return (atan((p2y-p1y)/(p2x-p1x)));
    else if(((p2y-p1y)>=0)&&((p2x-p1x)<0))         //2nd quadrant
        return (3.14159+atan((p2y-p1y)/(p2x-p1x)));
    else if(((p2y-p1y)<0)&&((p2x-p1x)<0))           //3rd quadrant
        return (3.14159+atan((p2y-p1y)/(p2x-p1x)));
    else if(((p2y-p1y)<0)&&((p2x-p1x)>=0))          //4th quadrant
        return ((2*3.14159)+atan((p2y-p1y)/(p2x-p1x)));
    else
        return (0);
}

