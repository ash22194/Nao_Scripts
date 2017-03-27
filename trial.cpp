// map::count

#include "functions.h"
#include <fstream>
// #include <iostream>
// #include <vector>

using namespace Eigen;
class tri
{
private:
  std::vector<int> v1;
public:
  tri(){};
  bool tp ()
  {
    std::vector<int> t;
    t.push_back(10);
    v1 = t;
  };
  void print_v1()
  {
    for (int i=0;i<v1.size();i++)
    {
      std::cout<<v1[i]<<std::endl;
    }
  };

};

int main ()
{
  std::vector<Vector3d> path;
  std::vector<Vector3d> action;
  // std::cout<<"Field : "<<f_s(0)<<","<<f_s(1)<<std::endl;
  // Vector3i state(-15,8,6);
  std::vector<Vector3i> neighbours;
  std::vector<double> h;
  std::map<Vector3i,bool,cmpV3i > obstacles;
  ObstacleMap(obstacles);
  // Vector3i v(0,0,1);
  // obstacles[v] = true;
  // std::cout<<"Ob : "<<obstacles.find(v)->second<<std::endl;
  // std::map<Vector3i,bool,cmpV3i >::iterator ob_it = obstacles.begin();
  // ob_it->second = false;
  // obstacles[v] = false;
  // std::cout<<"Ob : "<<obstacles.begin()->second<<std::endl;
  
  // std::cout<<"Floor : "<<std::floor(100/200)<<std::endl;
  // it = obstacles.find()
  // std::vector<char> v_char;
  // v_char.push_back('a');
  // v_char.push_back('b');
  // v_char.push_back('c');
  // v_char.push_back('d');

  // std::cout<<v_char.back()<<std::endl;
  // compute_neighbours(state, neighbours, h,obstacles);
  //   for (int i=0; i<neighbours.size();i++)
  //   {
  //       std::cout<<neighbours[i](0)<<","<<neighbours[i](1)<<","<<neighbours[i](2)<<std::endl;
  //   }
  // std::ofstream myfile;
  // myfile.open("path.txt");
  // ObstacleList parameters;
  // // for (int i = 0;i < 3;i++)
  // // {
  // //   std::cout<<parameters.Obst[i].center<<std::endl;
  // // }
  // // std::cout<<round(-1151/100)<<std::endl;
  // // std::map<int,bool>ob;
  // // ob[1] = true;
  // // ob[0] = true;
  // // ob[2] = true;
  // // std::cout<<(ob[2])<<std::endl;
  path.erase(path.begin(),path.end());
  Vector3i start_grid(std::floor((start(0) + g_s(0)/2)/g_s(0)),std::floor((start(1)+ g_s(1)/2)/g_s(1)),round((start(2))/g_s(2)));
    if (start(2) < 0)
    {
        start_grid(2) = round((start(2)+2*PI)/g_s(2));
    }
  //   Vector3i st = start_grid;
    // st[2] = 0;
  // std::map<Vector3i,bool,cmpV3i >::iterator it = obstacles.begin();
  // while(it!=obstacles.end())
  //   {   
  //       std::cout<<"("<<(it->first)[0]*g_s(0)<<","<<(it->first)[1]*g_s(1)<<","<<(it->first)[2]*g_s(2)<<")"<<std::endl;
  //       it++;
  //   }
  // if (it!=obstacles.end())
  // {
  //   std::cout<<"Start is blocked at :"<<(it->first)<<","<<(it->second)<<std::endl;
  // }   

  // compute_neighbours(start_grid,neighbours,h,obstacles);
  // std::cout<<"Neighbours"<<std::endl;
  // for (int i=0; i<neighbours.size(); i++)
  // {
  //   std::cout<<neighbours[i].x()*g_s(0)<<","<<neighbours[i].y()*g_s(1)<<","<<neighbours[i].z()*g_s(2)<<std::endl;
  // }

  bool success = plan(path,action);
  if (success)
  {
    std::cout<<"Path"<<std::endl;
    for (int i = 0; i < path.size(); i++)
    {
      std::cout<<"("<<path[i](0)<<","<<path[i](1)<<","<<path[i](2)<<")"<<std::endl;  
    }
    for (int i = 0; i < action.size(); i++)
    {
      std::cout<<"("<<action[i](0)<<","<<action[i](1)<<","<<action[i](2)<<")"<<std::endl;  
    }
    path.erase(path.begin(),path.end());
    action.erase(action.begin(),action.end());
  }
  else
  {
    std::cout<<"No path found!"<<std::endl;
  }
  // std::map<double,std::map<Vector3i,bool,cmpV3i> > op;
  // Vector3i v(10,0,0);
  // (op[1.2])[v] = true;
  // v(2) = 1;
  // (op[1.21])[v] = true;
  // v(2) = -1;
  // (op[1.19])[v] = true;
  // v(2) = 3;
  // (op[1.2])[v] = true;
  // v(2) = 4;
  // (op[1.1901])[v] = true;

  // std::map<double,std::map<Vector3i,bool,cmpV3i> >::iterator it = op.begin();
  
  // while(it!=op.end())
  // { std::map<Vector3i,bool,cmpV3i>::iterator it1 = (it->second).begin();
  //   std::cout<<"F : "<<(it->first)<<std::endl;
  //   while(it1!=(it->second).end())
  //   {
  //     Vector3i v = (it1->first);
  //     std::cout<<v(0)<<","<<v(1)<<","<<v(2)<<std::endl;
  //     it1++;
  //   }
  //   it++;
  // }
  
  // success = plan(path);
  // if (success)
  //   {
  //     std::cout<<"second attempt"<<path.size()<<std::endl;
  //   }
  // myfile<<"Size : "<<path.size()<<"\n";

//   // Vector3i v(2,3,4);
//   // std::cout<<heuristic(v)<<std::endl;
//   // Vector3i state(-30,23,0); 
//   // std::vector<Vector3i> neighbours; 
//   // std::vector<double> h;
//   // compute_neighbours(state,neighbours,h);
//   // for (int i = 0;i < neighbours.size();i++)
//   // {
//   //   std::cout<<"State "<<(neighbours[i])(0)<<","<<(neighbours[i])(1)<<","<<(neighbours[i])(2)<<" H "<<h[i]<<std::endl;

//   // }
// //   tri tri_;
// //   tri_.tp();
// //   std::cout<<"V1 :"<<std::endl;
// //   tri_.print_v1();
//   std::map<char,std::map<char,bool> > mymap;
// //   std::vector<int> v1,v2,v3,v4,v5;
// //   // std::cout<<"Print"<<std::endl;
// //   v1.push_back(1);
// //   v1.push_back(2);
// //   v1.push_back(3);
// //   v2.push_back(5);
// //   v2.push_back(6);
// //   // std::cout<<"Print"<<std::endl;
//   (mymap['c'])['b'] = false;
//   (mymap['b'])['a'] = false;
//   (mymap['b'])['c'] = false;
//   (mymap['a'])['a'] = true;
// //   // mymap['b'].push_back(7);
// //   (mymap['c'])['a'] = false;
// //   // v3 = mymap['b'];
// //   // v3.push_back(7);
// //   // v4.push_back(0);
// //   // mymap['b'] = v3;
// //   // mymap['c'] = v4;
// //   // std::cout<<"Print"<<std::endl;
// //   // v5 = mymap['a'];
// //   // int p = (mymap.begin()->second).back();
// //   // Vector3i a(1,2,3);
// //   // std::cout<<sqrt(a(0)*a(0) + a(1)*a(1) + a(2)*a(2))<<std::endl;
//   std::map<char,std::map<char,bool> >::iterator it = mymap.begin();
// //   // int p = (it->second).back();
// //   // it--;
//   std::map<char,bool>::iterator it_ = (it->second).begin();
//   std::cout<<"First "<<it->first<<" First_first "<<(it_)->first<<" First_second "<<((it_)->second)<<std::endl;
//   (it->second).erase(it_);
// if ((it->second).size()==0)
//   {
//     mymap.erase(it);
//   }
//   it = mymap.begin();
// //   // int p = (it->second).back();
//   it_ = (it->second).begin();
//   std::cout<<"Size "<<((mymap.begin())->second).size()<<std::endl;
//   std::cout<<"First "<<it->first<<" First_first "<<(it_)->first<<" First_second "<<((it_)->second)<<std::endl;
//   (it->second).erase(it_);
//   it_ = (it->second).begin();
//   std::cout<<"First "<<it->first<<" First_first "<<(it_)->first<<" First_second "<<((it_)->second)<<std::endl;
//   std::cout<<"Size "<<mymap.size()<<std::endl;
//   // std::cout<<"Round"<<round(1.98)<<std::endl;
//   //   v5 = mymap['b'];
//   // std::cout<<"b"<<v5[2]<<std::endl;
//   //   v5 = mymap['c'];
//   // std::cout<<"c"<<v5[0]<<std::endl;
//     // std::cout<<int(-2350/100)<<std::endl;
//     // std::vector<int> v;
//     // std::vector<int>::iterator it__;
//     // // std::cout<<(it__==NULL)<<std::endl;
//     // v.push_back(10);
//     // it__ = v.begin();
//     // int i = 0;
//     // int j = 0;
//     // std::cout<<((i==0)&(j==0))<<std::endl;
//     //     std::cout<<((i==0)&&(j==0))<<std::endl;
//     // v.erase(v.begin(),v.end());
//     // // std::cout<<(v.size())<<std::endl;
//     // std::map<Vector3i,bool,cmpV3i > obstacles;
//     // int num_obstacles = 1;
//     // Vector3f g_s(100,100,PI/4);
//     // Vector2f field(7300,4600);
//     // Vector2i f_s(int(field(0)/g_s(0))-1,int(field(1)/g_s(1))-1);
//     // // Grid coordinates (x_i,y_i) x_i <= f_s(0) and y_i <= f_s(1)
//     // // std::vector<Vector2f> corners;
//     // for (int k=0; k<num_obstacles; k++)
//     // {
//     //     Vector2f c(-2300,0);
//     //     Vector2f s(500,200);
//     //     double ori = PI/4;
//     //     // Vector2f c1(c(0) + s(0)/2*cos(ori) - s(1)/2*sin(ori), c(1) + s(0)/2*sin(ori) + s(1)/2*cos(ori));
//     //     // Vector2f c2(c(0) - s(0)/2*cos(ori) - s(1)/2*sin(ori), c(1) - s(0)/2*sin(ori) + s(1)/2*cos(ori));
//     //     // Vector2f c3(c(0) - s(0)/2*cos(ori) + s(1)/2*sin(ori), c(1) - s(0)/2*sin(ori) - s(1)/2*cos(ori));
//     //     // Vector2f c4(c(0) + s(0)/2*cos(ori) + s(1)/2*sin(ori), c(1) + s(0)/2*sin(ori) - s(1)/2*cos(ori));
//     //     double r = sqrt(pow(s(0)/2,2) + pow(s(1)/2,2));
//     //     for (int i = int((c(0)-r + g_s(0)/2)/g_s(0));i<=int((c(0)+r + g_s(1)/2)/g_s(0));i++)
//     //     {
//     //         for (int j = int((c(1)-r+ g_s(0)/2)/g_s(1));j<=int((c(1)+r+ g_s(1)/2)/g_s(1));j++)
//     //         {
//     //             if ( sqrt(pow(i*g_s(0) - c(0),2) + pow(j*g_s(1) - c(1),2))  < r)
//     //             {   
//     //                 Vector3i v(i,j,0);
//     //                 std::cout<<"\n("<<v(0)<<","<<v(1)<<")"<<std::endl;
//     //                 obstacles[v] = true;
//     //             }
//     //         }
//     //     }
//     // }

  return 0;
}