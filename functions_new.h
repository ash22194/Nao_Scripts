#include <iostream>
#include <math.h>
#include <map>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <fstream>

using namespace Eigen;

extern double PI = 3.14159265359;

extern Vector3d g_s(200,200,PI/4);
extern Vector2i f_s(round(7300/g_s(0)),round(4600/g_s(1)));
// extern Vector2i f_s(round(400/g_s(0)),round(400/g_s(1)));

extern Vector3d target(200,600,1.5708);
// extern Vector3d start(-3002.04, 1495.7,-PI/2);
extern Vector3d start(0,0,-3.1416);
// extern Vector3d target(199,0,0);
// extern Vector3d start(199,299,4);

struct Obstacles
            {
                Vector2d center;
                Vector2d size;
                double orientation;
                // Obstacles(Vector2d c, Vector2d s, double o) : center(c), size(s), orientation(o) {}  
            };

extern Obstacles ob1 = {
    Vector2d(-2300,0),
    Vector2d(200,200),
    0
};
extern Obstacles ob2 = {
    Vector2d(-2000,-620),
    Vector2d(700,550),
    1.5708
};
// extern Obstacles ob3 = {
//     Vector2d(-2597,-1016),
//     Vector2d(650,560),
//     PI/4
// };

struct ObstacleList
        {
            std::vector<Obstacles> Obst;
            ObstacleList() 
            {
                Obst.push_back(ob1);
                Obst.push_back(ob2);
                // Obst.push_back(ob3);
            }; 
        };

extern ObstacleList parameters;
// ob1.center(-3163,-450);
// ob1.size(650,560);
// ob1.orientation = PI/4;

// ob2.center(-2880,-733);
// ob2.size(650,560);
// ob2.orientation = PI/4;

// ob3.center(-2597,-1016);
// ob3.size(650,560);
// ob3.orientation = PI/4;
// extern ObstacleList parameters = {
//     std::vector<Obstacles>(ob1,ob2,ob3)
// };

// parameters.Obst.push_back(ob1);
// parameters.Obst.push_back(ob2);
// parameters.Obst.push_back(ob3);

struct cmpV3i 
    {
        bool operator()(const Vector3i& a, const Vector3i& b) const 
        {
            if (a(0)==b(0))
            {
                if (a(1)==b(1))
                {
                    return a(2) < b(2);
                }
                else
                {
                    return a(1) < b(1);
                }
            }
            else
            {
                return a(0) < b(0);
            }
        }
    };

void compute_neighbours(Vector3i state, std::vector<Vector3i>& neighbours, std::vector<double>& h, std::map<Vector3i,bool,cmpV3i >& obstacles)//, std::vector<Vector3i>& moves)
  {
    // double PI;
    double f_t_c = 1;
    double r_c = 1;
    double s_t_c = f_t_c*2;
    double b_t_c = f_t_c*3;
    int x = state(0);
    int y = state(1);
    double theta = (state(2)*PI/4);
    int orientation = state(2);

    Vector3i state_ = state;
    state_.z() = 0;
    if (obstacles.find(state_)!=obstacles.end())
    {
        std::cout<<"Start position is blocked"<<std::endl;
        return;
    }
    if (orientation==8)
    {
        theta = 0;
        orientation = 0;
    }
    double theta_1 = theta + PI/4;
    double theta_2 = theta - PI/4;
    if (theta_2 < 0)
    {
        theta_2 = theta_2 + 2*PI;
    }
    if (theta_1 > 6.2)
    {
        theta_1 = 0;
    }

    if (orientation%2==1)
    {
        f_t_c = f_t_c*sqrt(2);
        s_t_c = s_t_c*sqrt(2);
        b_t_c = b_t_c*sqrt(2);

        for(int i = -1; i < 2; i++)
        {
            for (int j = -1; j < 2; j++)
            {
                int x_ = x + i;
                int y_ = y + j;
                Vector3i neigh(x_,y_,0);
                // std::map<Vector2i,bool>::iterator obs_it = obstacles.find(neigh);
                std::map<Vector3i,bool>::iterator obs_it = obstacles.find(neigh);

                bool blocked = (obs_it!=obstacles.end());
                // std::cout<<"Candidate : "<<neigh.x()*g_s(0)<<","<<neigh.y()*g_s(1)<<","<<neigh.z()*g_s(2)<<" Blocked : "<<blocked<<"Number of obstacles : "<<obstacles.size()<<std::endl;
                // bool blocked = false;

                if (!((x_ < -f_s(0)/2) || (x_ > f_s(0)/2) || (y_ < -f_s(1)/2) || (y_ > f_s(1)/2) || (blocked)))
                {   
                    if (std::abs(i+j)%2==1)
                    {
                        continue;
                    }
                    else
                    {
                        if ((i==0) && (j==0))
                        {
                            continue;
                        }
                        // std::cout<<i<<","<<j<<","<<(i+j)%2<<std::endl;
                        Vector3i state_(x_,y_,orientation);
                        neighbours.push_back(state_);
                        double cost;
                        // move.push_back(Vector3i v(x_ - x,y_ - y,0));
                        if (std::abs(i*cos(theta) + j*sin(theta)) < 0.1)
                        {
                            cost = s_t_c;
                        }
                        else if (i*cos(theta) + j*sin(theta) > 0)
                        {
                            cost = f_t_c;
                        }
                        else if (i*cos(theta) + j*sin(theta) < 0)
                        {
                            cost = b_t_c;
                        }  

                        h.push_back(cost);
                    }
                }
            }
        }
    }
    else
    {
        for(int i = -1; i < 2; i++)
        {
            for(int j = -1; j < 2; j++)
            {
                int x_ = x + i;
                int y_ = y + j;
                Vector3i neigh(x_,y_,0);
                // std::map<Vector2i,bool>::iterator obs_it = obstacles.find(neigh);
                std::map<Vector3i,bool>::iterator obs_it = obstacles.find(neigh);
                bool blocked = (obs_it!=obstacles.end());
                // std::cout<<"Candidate : "<<neigh<<" Blocked : "<<blocked<<"Number of obstacles : "<<obstacles.size()<<std::endl;
                // bool blocked = false;
                if (!((x_ < -f_s(0)/2) || (x_ > f_s(0)/2) || (y_ < -f_s(1)/2) || (y_ > f_s(1)/2) || (blocked)))
                {
                    if (std::abs(i+j)%2==0)
                    {
                        continue;
                    }
                    else
                    {
                        Vector3i state_(x_,y_,orientation);
                        neighbours.push_back(state_);
                        double cost;
                        // move.push_back(Vector3i v(x_ - x,y_ - y,0));
                        if (std::abs(i*cos(theta) + j*sin(theta)) < 0.1)
                        {
                            cost = s_t_c;
                        }
                        else if (i*cos(theta) + j*sin(theta) > 0)
                        {
                            cost = f_t_c;
                        }
                        else if (i*cos(theta) + j*sin(theta) < 0)
                        {
                            cost = b_t_c;
                        }
                        h.push_back(cost);
                    }
                }
            }
        }   
    }

    neighbours.push_back(Vector3i(x,y,round(theta_1/(PI/4))));
    h.push_back(r_c*PI/4);
    neighbours.push_back(Vector3i(x,y,round(theta_2/(PI/4))));
    h.push_back((r_c*PI/4));

};

double heuristic(Vector3i state_grid)
{   
    // double PI;
    Vector3i t(std::floor((target(0) + g_s(0)/2)/g_s(0)),std::floor((target(1) + g_s(1)/2)/g_s(1)),round((target(2))/g_s(2)));
    // std::cout<<"t\n"<<PI<<std::endl;
    Vector3d diff(state_grid(0) - t(0),state_grid(1) - t(1),state_grid(2) - t(2));
    double angle_diff = std::abs(state_grid(2) - t(2))*PI/4;
    diff(2) = std::min(angle_diff,2*PI - angle_diff);
    // std::cout<<"diff\n"<<diff<<std::endl;
    double h = sqrt(diff(0)*diff(0) + diff(1)*diff(1) + diff(2)*diff(2)); 
    return h;
};

void ObstacleMap(std::map<Vector3i,bool,cmpV3i >& obstacles)
  {
    // double PI;
    ObstacleList parameters;
    int num_obstacles = parameters.Obst.size();
    for (int k=0; k<num_obstacles; k++)
    {
        Vector2d c = parameters.Obst[k].center;
        Vector2d s = parameters.Obst[k].size;
        // double ori = parameters.obs.Obst[k].orientation;
        // Vector2d c1(c(0) + s(0)/2*cos(ori) - s(1)/2*sin(ori), c(1) + s(0)/2*sin(ori) + s(1)/2*cos(ori));
        // Vector2d c2(c(0) - s(0)/2*cos(ori) - s(1)/2*sin(ori), c(1) - s(0)/2*sin(ori) + s(1)/2*cos(ori));
        // Vector2d c3(c(0) - s(0)/2*cos(ori) + s(1)/2*sin(ori), c(1) - s(0)/2*sin(ori) - s(1)/2*cos(ori));
        // Vector2d c4(c(0) + s(0)/2*cos(ori) + s(1)/2*sin(ori), c(1) + s(0)/2*sin(ori) - s(1)/2*cos(ori));
        double r = sqrt(pow(s(0)/2,2) + pow(s(1)/2,2));
        std::cout<<r<<std::endl;
        for (int i = round((c(0) - r + g_s(0)/2)/g_s(0));i<round((c(0) + r + g_s(0)/2)/g_s(0));i++)
        {
            for (int j = round((c(1) - r + g_s(1)/2)/g_s(1));j<round((c(1) + r + g_s(1)/2)/g_s(1));j++)
            {
                if ( sqrt(pow(i*g_s(0) - c(0),2) + pow(j*g_s(1) - c(1),2))  < r)
                {   
                    Vector3i v(i,j,0);
                    obstacles[v] = true;
                }
            }
        }
    }
  };

bool plan(std::vector<Vector3d>& path)
  // { 
  //   // double PI;
  //   Vector3i start_grid(round((start(0) + g_s(0)/2)/g_s(0)),round((start(1)+ g_s(1)/2)/g_s(1)),round((start(2))/g_s(2)));
  //   if (start(2) < 0)
  //   {
  //       start_grid(2) = round((start(2)+2*PI)/g_s(2));
  //   }   
  //   Vector3i target_grid(round((target(0)+ g_s(0)/2)/g_s(0)),round((target(1)+ g_s(1)/2)/g_s(1)),round((target(2))/g_s(2)));
  //   if (target(2) < 0)
  //   {
  //       target_grid(2) = round((target(2)+2*PI)/g_s(2));
  //   }
  //   std::cout<<"Start Grid : "<<start_grid<<std::endl;
  //   std::cout<<"Target Grid : "<<target_grid<<std::endl;
  //   /*
  //   Create grid
  //   Convert start and target locations to grid coordinates
  //   Identify inaccessible grid coordinates using obstaacle information
  //   */

  //   // A*
  //   std::map<Vector3i,bool,cmpV3i > obstacles;
  //   ObstacleMap(obstacles);
  //   std::map<Vector3i,bool,cmpV3i >::iterator it = obstacles.begin();
  //   std::cout<<"Obstacles"<<std::endl;
  //   while(it!=obstacles.end())
  //   {   
  //       std::cout<<"("<<(it->first)[0]*g_s(0)<<","<<(it->first)[1]*g_s(1)<<","<<(it->first)[2]*g_s(2)<<")"<<std::endl;
  //       it++;
  //   }
  //   std::map<double, std::map<Vector3i,bool,cmpV3i> > open_states;
  //   std::map<Vector3i, double, cmpV3i > closed_states;
  //   std::map<Vector3i, double, cmpV3i > g;
  //   std::map<Vector3i, Vector3i, cmpV3i > tree,moves;
  //   Vector3i next = start_grid;
  //   g[next] = 0;
  //   bool no_path = false;
  //   int count_ = 0;

  //   while(heuristic(next)!=0)
  //   {
  //       count_++;
  //       double g_ = g[next];
  //       double f = g_ + heuristic(next);
  //       // std::cout<<"("<<next(0)<<","<<next(1)<<","<<next(2)<<")"<<f<<std::endl;
  //       closed_states[next] = f;
  //       std::vector<Vector3i> neighbours;
  //       std::vector<double> one_step_cost;
  //       compute_neighbours(next,neighbours,one_step_cost,obstacles);

  //       if ((neighbours.size()==0) || (one_step_cost.size()==0) || (one_step_cost.size()!=neighbours.size()))
  //       {
  //           std::cout<<"Check compute neighbours"<<std::endl;
  //       }

  //       for (int i = 0; i < neighbours.size(); i++)
  //       {
  //           if (!(closed_states.count(neighbours[i])>0))
  //           {
  //               g[neighbours[i]] = g_ + one_step_cost[i];
  //               double h = heuristic(neighbours[i]);
  //               f = h + g[neighbours[i]];
  //               (open_states[f])[neighbours[i]] = true;
  //               tree[neighbours[i]] = next;
  //           }
  //       }
        
  //       if(open_states.size()==0)
  //       {
  //           no_path = true;
  //           break;
  //       }
  //       else
  //       {   
  //           std::map<double,std::map<Vector3i,bool,cmpV3i> >::iterator it = open_states.begin();
  //           std::map<Vector3i,bool,cmpV3i>::iterator it_ = (it->second).begin();
  //           while(closed_states.count(it_->first)>0)
  //           {
  //               (it->second).erase(it_);
  //               it_ = (it->second).begin();  
  //               if (it_ == (it->second).end())
  //               {
  //                   open_states.erase(it);
  //                   if(open_states.size()==0)
  //                   {
  //                       no_path = true;
  //                       break;
  //                   }
  //                   else
  //                   {
  //                       it = open_states.begin();
  //                       it_ = (it->second).begin();
  //                   }
  //               }
  //           }

  //           if (no_path)
  //           {
  //               break;
  //           }

  //           next = (it_->first);

  //           if ((it->second).size()<=1)
  //           {
  //               open_states.erase(it);
  //           }
  //           else
  //           {
  //               (it->second).erase(it_);
  //           }
  //           moves[next] = tree[next];
  //       }
  //       neighbours.erase(neighbours.begin(),neighbours.end());
  //       one_step_cost.erase(one_step_cost.begin(),one_step_cost.end());
  //   }

  //   if (!no_path)
  //   {
  //       while(next!=start_grid)
  //       {
  //           Vector3i back_trace = next;
  //           Vector3i check = next;
  //           check(2) = 0;
  //           if (obstacles[check])
  //           {
  //               std::cout<<"Collision!"<<std::endl;
  //           }
  //           Vector3d b_t(back_trace(0)*g_s(0),back_trace(1)*g_s(1),back_trace(2)*g_s(2));
  //           if(b_t(2)>PI)
  //           {
  //               b_t(2) = b_t(2) - 2*PI;
  //           }
  //           path.push_back(b_t);
  //           next = moves[next];
  //       }
  //       Vector3i back_trace = next;
  //       Vector3d b_t(back_trace(0)*g_s(0),back_trace(1)*g_s(1),back_trace(2)*g_s(2));
  //       if(b_t(2)>PI)
  //           {
  //               b_t(2) = b_t(2) - 2*PI;
  //           }
  //       path.push_back(b_t);
  //       return true;
  //   }
  //   else
  //   {   
  //       return false;
  //   }
  // };
{
    std::cout<<"Plan"<<std::endl;
    std::ofstream myfile,tree_file;
    myfile.open("path.txt");
    tree_file.open("tree.txt");
    myfile<<"Size : "<<path.size()<<"\n";

    path.erase(path.begin(),path.end());
    // Pose2f start = theRobotPose.inversePose.inverse();
    myfile<<"Start : "<<start(0)<<","<<start(1)<<","<<start(2)<<std::endl; 
    myfile<<"Target : "<<target(0)<<","<<target(1)<<","<<target(2)<<std::endl;
   
    // Pose2f target = parameters.target_location;
    // double start_angle = start.rotation;
    // if (start_angle < 0)
    // {
    //     start_angle = start_angle + 2*PI;
    // }
    // double target_angle = target.rotation;
    // if (target_angle < 0)
    // {
    //     target_angle = target_angle + 2*PI;
    // }
    // Vector3d g_s = parameters.grid_size;
    std::map<Vector3i,bool,cmpV3i > obstacles;
    ObstacleMap(obstacles);
    std::map<Vector3i,bool,cmpV3i >::iterator it = obstacles.begin();
    std::cout<<"Obstacles"<<std::endl;
    // while(it!=obstacles.end())
    // {   
    //     std::cout<<"("<<(it->first)[0]*g_s(0)<<","<<(it->first)[1]*g_s(1)<<","<<(it->first)[2]*g_s(2)<<")"<<std::endl;
    //     it++;
    // }

    Vector3i start_grid(std::floor((start(0) + g_s(0)/2)/g_s(0)),std::floor((start(1)+ g_s(1)/2)/g_s(1)),round((start(2))/g_s(2)));
    if (start(2) < 0)
    {
        start_grid(2) = round((start(2)+2*PI)/g_s(2));
    }   
    Vector3i target_grid(std::floor((target(0)+ g_s(0)/2)/g_s(0)),std::floor((target(1)+ g_s(1)/2)/g_s(1)),round((target(2))/g_s(2)));
    if (target(2) < 0)
    {
        target_grid(2) = round((target(2)+2*PI)/g_s(2));
    }
    // Vector3i start_grid(round((start.translation.x() + g_s(0)/2)/g_s(0)),round((start.translation.y()+ g_s(1)/2)/g_s(1)),round((start_angle)/g_s(2)));
    // Vector3i target_grid(round((target.translation.x()+ g_s(0)/2)/g_s(0)),round((target.translation.y()+ g_s(1)/2)/g_s(1)),round((target_angle)/g_s(2)));
    std::cout<<"Start Grid : "<<start_grid<<std::endl;
    std::cout<<"Target Grid : "<<target_grid<<std::endl;

    myfile<<"S_grid : "<<start_grid(0)<<","<<start_grid(1)<<","<<start_grid(2)<<","<<heuristic(start_grid)<<std::endl;
    myfile<<"T_grid : "<<target_grid(0)<<","<<target_grid(1)<<","<<target_grid(2)<<","<<heuristic(target_grid)<<std::endl;

    /*
    Create grid
    Convert start and target locations to grid coordinates
    Identify inaccessible grid coordinates using obstaacle information
    */

    // A*
    std::map<double, std::map<Vector3i,bool,cmpV3i> > open_states;
    std::map<Vector3i, double, cmpV3i > closed_states;
    std::map<Vector3i, double, cmpV3i > g;
    std::map<Vector3i, double, cmpV3i > f_values;
    std::map<Vector3i, Vector3i, cmpV3i > tree,moves;
    Vector3i next = start_grid;
    g[next] = 0;
    bool no_path = false;
    int count_ = 0;
    while(heuristic(next)!=0)
    {
        count_++;
        double g_ = g[next];
        double f = g_ + heuristic(next);
        f_values[next] = f;
        closed_states[next] = f;
        std::map<Vector3i, double, cmpV3i >::iterator closed_states_it = closed_states.find(next);
        myfile<<"Next : \n";
        myfile<<next(0)<<","<<next(1)<<","<<next(2)<<","<<" f : "<<((closed_states_it)->second)<<", g : "<<g_<<", h : "<<heuristic(next)<<"\n";

        std::vector<Vector3i> neighbours;
        std::vector<double> one_step_cost;
        compute_neighbours(next,neighbours,one_step_cost,obstacles);

        if ((neighbours.size()==0) || (one_step_cost.size()==0) || (one_step_cost.size()!=neighbours.size()))
        {
            // std::cout<<"Check compute neighbours"<<std::endl;
        }
        // int count = 0;
        for (int i = 0; i < neighbours.size(); i++)
        {
            // myfile<<neighbours[i](0)<<","<<neighbours[i](1)<<","<<neighbours[i](2)<<","<<closed_states.count(neighbours[i])<<std::endl;

            if (!(closed_states.count(neighbours[i])>0))
            {
                // count++;
                g[neighbours[i]] = g_ + one_step_cost[i];
                double h = heuristic(neighbours[i]);
                f = h + g[neighbours[i]];
                double f_val = f_values.find(neighbours[i])->second;
                if (f_values.find(neighbours[i])==f_values.end())
                {
                    tree[neighbours[i]] = next;
                    f_values[neighbours[i]] = f;
                    (open_states[f])[neighbours[i]] = true;
                }
                else if (f_val > f)
                {
                    tree[neighbours[i]] = next;
                    f_values[neighbours[i]] = f;
                    std::map<double,std::map<Vector3i,bool,cmpV3i> >::iterator it_open = open_states.find(f_val);
                    std::map<Vector3i,bool,cmpV3i>::iterator it_open_sec = (it_open->second).find(neighbours[i]);
                    
                    if (it_open_sec!=(it_open->second).end())
                    {
                        (it_open->second).erase(it_open_sec);   
                        if (it_open->second.size() < 1)
                        {
                            open_states.erase(it_open);
                        }
                    }  
                }
            }
        }
        myfile<<"Open_states : \n";
        std::map<double,std::map<Vector3i,bool,cmpV3i> >::iterator it1 = open_states.begin();
        while(it1!=open_states.end())
        {
            std::map<Vector3i,bool,cmpV3i>::iterator it2 = (it1->second).begin();
            myfile<<"F : "<<(it1->first)<<"\n";
            while(it2!=(it1->second).end())
            {
                myfile<<(it2->first)(0)<<","<<(it2->first)(1)<<","<<(it2->first)(2)<<"\n";
                it2++;
            }
            it1++;
        }
        std::map<Vector3i, double, cmpV3i>::iterator it1_c = closed_states.begin();
        myfile<<"Closed_states : \n";
        while(it1_c!=closed_states.end())
        {
                
                myfile<<"F : "<<(it1_c->second)<<","<<(it1_c->first)(0)<<","<<(it1_c->first)(1)<<","<<(it1_c->first)(2)<<"\n";
            it1_c++;
        }
        
        // if (count==0)
        // {
        //     // std::cout<<"No new neighbours added from "<<next(0)<<","<<next(1)<<","<<next(2)<<std::endl;
        // }

        if(open_states.size()==0)
        {
            no_path = true;
            std::cout<<"exit 1"<<std::endl;
            break;
        }
        else
        {   
            std::map<double,std::map<Vector3i,bool,cmpV3i> >::iterator it = open_states.begin();
            std::map<Vector3i,bool,cmpV3i>::iterator it_ = (it->second).begin();
            while(closed_states.count(it_->first)>0)
            {
                // std::cout<<"Already closed"<<std::endl;
                // closed<<(it_->first)(0)<<","<<(it_->first)(1)<<","<<(it_->first)(2)<<","<<count_<<"\n";
                (it->second).erase(it_);
                it_ = (it->second).begin();  
                if (it_ == (it->second).end())
                {
                    open_states.erase(it);
                    if(open_states.size()==0)
                    {
                        no_path = true;
                        std::cout<<"exit 2"<<std::endl;
                        break;
                    }
                    else
                    {
                        it = open_states.begin();
                        it_ = (it->second).begin();
                    }
                }
            }
            if (no_path)
            {
                std::cout<<"exit 2.2"<<std::endl;
                break;
            }

            next = (it_->first);
            if ((it->second).size()<=1)
            {
                open_states.erase(it);
            }
            else
            {
                (it->second).erase(it_);
            }
            moves[next] = tree.find(next)->second;
        }
        neighbours.erase(neighbours.begin(),neighbours.end());
        one_step_cost.erase(one_step_cost.begin(),one_step_cost.end());
    }
    std::map<Vector3i,Vector3i,cmpV3i>::iterator tree_it = tree.begin();
    while(tree_it!=tree.end())
    {
        tree_file<<"F : "<<f_values.find(tree_it->first)->second<<" Node : "<<tree_it->first.x()<<","<<tree_it->first.y()<<","<<tree_it->first.z()<<
        " Parent : "<<tree_it->second.x()<<","<<tree_it->second.y()<<","<<tree_it->second.z()<<std::endl; 
        tree_it++;
        
    }
    std::cout<<"Done exploring!"<<std::endl;

    if (!no_path)
    {   std::cout<<next(0)<<","<<next(1)<<","<<next(2)<<std::endl;
        while(next!=start_grid)
        {
            Vector3i back_trace = next;
            Vector3i check = next;
            check(2) = 0;
            if (obstacles.find(check)!=obstacles.end())
            {
                std::cout<<"Collision!"<<std::endl;
            }
            // double angle = back_trace(2)*g_s(2);
            // if(angle > PI)
            // {
            //     angle = angle - 2*PI;
            // }
            // Pose2f b_t(angle,back_trace(0)*g_s(0),back_trace(1)*g_s(1));
            Vector3d b_t(back_trace(0)*g_s(0),back_trace(1)*g_s(1),back_trace(2)*g_s(2));
            if(b_t(2)>PI)
            {
                b_t(2) = b_t(2) - 2*PI;
            }
            path.push_back(b_t);
            // myfile<<"("<<b_t(0)<<","<<b_t(1)<<","<<b_t(2)<<")"<<std::endl;
            next = moves.find(next)->second;
        }
        Vector3i back_trace = next;
        Vector3d b_t(back_trace(0)*g_s(0),back_trace(1)*g_s(1),back_trace(2)*g_s(2));
            if(b_t(2)>PI)
            {
                b_t(2) = b_t(2) - 2*PI;
            }
            path.push_back(b_t);
            
        // double angle = back_trace(2)*g_s(2);
        // if(angle > PI)
        //     {
        //         angle = angle - 2*PI;
        //     }
        // Pose2f b_t(angle,back_trace(0)*g_s(0),back_trace(1)*g_s(1));
       // myfile<<"("<<b_t(0)<<","<<b_t(1)<<","<<b_t(2)<<")"<<std::endl;
            
        std::reverse(path.begin(),path.end());
        open_states.erase(open_states.begin(),open_states.end());
        closed_states.erase(closed_states.begin(),closed_states.end());
        g.erase(g.begin(),g.end());
        tree.erase(tree.begin(),tree.end());
        moves.erase(moves.begin(),moves.end());
        return true;
    }
    else
    {   open_states.erase(open_states.begin(),open_states.end());
        closed_states.erase(closed_states.begin(),closed_states.end());
        g.erase(g.begin(),g.end());
        tree.erase(tree.begin(),tree.end());
        moves.erase(moves.begin(),moves.end());
        return false;
    }
  };



