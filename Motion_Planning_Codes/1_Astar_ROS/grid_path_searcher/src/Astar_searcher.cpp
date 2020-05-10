#include "Astar_searcher.h"
#include <iostream>
using namespace std;
using namespace Eigen;
void AstarPathFinder::initGridMap(double _resolution, Vector3d global_xyz_l, Vector3d global_xyz_u, int max_x_id, int max_y_id, int max_z_id)
{   
    gl_xl = global_xyz_l(0);
    gl_yl = global_xyz_l(1);
    gl_zl = global_xyz_l(2);
    gl_xu = global_xyz_u(0);
    gl_yu = global_xyz_u(1);
    gl_zu = global_xyz_u(2);
    GLX_SIZE = max_x_id;
    GLY_SIZE = max_y_id;
    GLZ_SIZE = max_z_id;
    GLYZ_SIZE  = GLY_SIZE * GLZ_SIZE;
    GLXYZ_SIZE = GLX_SIZE * GLYZ_SIZE;
    resolution = _resolution;
    inv_resolution = 1.0 / _resolution;    
    data = new uint8_t[GLXYZ_SIZE];
    memset(data, 0, GLXYZ_SIZE * sizeof(uint8_t));   
    GridNodeMap = new GridNodePtr ** [GLX_SIZE];
    for(int i = 0; i < GLX_SIZE; i++){
        GridNodeMap[i] = new GridNodePtr * [GLY_SIZE];
        for(int j = 0; j < GLY_SIZE; j++){
            GridNodeMap[i][j] = new GridNodePtr [GLZ_SIZE];
            for( int k = 0; k < GLZ_SIZE;k++){
                Vector3i tmpIdx(i,j,k);
                Vector3d pos = gridIndex2coord(tmpIdx);
                GridNodeMap[i][j][k] = new GridNode(tmpIdx, pos);
            }
        }
    }
}
void AstarPathFinder::resetGrid(GridNodePtr ptr)
{
    ptr->id = 0;
    ptr->cameFrom = NULL;
    ptr->gScore = inf;
    ptr->fScore = inf;
}
void AstarPathFinder::resetUsedGrids()
{   
    for(int i=0; i < GLX_SIZE ; i++)
        for(int j=0; j < GLY_SIZE ; j++)
            for(int k=0; k < GLZ_SIZE ; k++)
                resetGrid(GridNodeMap[i][j][k]);
}
void AstarPathFinder::setObs(const double coord_x, const double coord_y, const double coord_z)
{   
    if( coord_x < gl_xl  || coord_y < gl_yl  || coord_z <  gl_zl || 
        coord_x >= gl_xu || coord_y >= gl_yu || coord_z >= gl_zu )
        return;
    int idx_x = static_cast<int>( (coord_x - gl_xl) * inv_resolution);
    int idx_y = static_cast<int>( (coord_y - gl_yl) * inv_resolution);
    int idx_z = static_cast<int>( (coord_z - gl_zl) * inv_resolution);      
    data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] = 1;
}
vector<Vector3d> AstarPathFinder::getVisitedNodes()
{   
    vector<Vector3d> visited_nodes;
    for(int i = 0; i < GLX_SIZE; i++)
        for(int j = 0; j < GLY_SIZE; j++)
            for(int k = 0; k < GLZ_SIZE; k++){   
                //if(GridNodeMap[i][j][k]->id != 0) // visualize all nodes in open and close list
                if(GridNodeMap[i][j][k]->id == -1)  // visualize nodes in close list only
                    visited_nodes.push_back(GridNodeMap[i][j][k]->coord);
            }
    ROS_WARN("visited_nodes size : %d", visited_nodes.size());
    return visited_nodes;
}
Vector3d AstarPathFinder::gridIndex2coord(const Vector3i & index) 
{
    Vector3d pt;
    pt(0) = ((double)index(0) + 0.5) * resolution + gl_xl;
    pt(1) = ((double)index(1) + 0.5) * resolution + gl_yl;
    pt(2) = ((double)index(2) + 0.5) * resolution + gl_zl;
    return pt;
}
Vector3i AstarPathFinder::coord2gridIndex(const Vector3d & pt) 
{
    Vector3i idx;
    idx <<  min( max( int( (pt(0) - gl_xl) * inv_resolution), 0), GLX_SIZE - 1),
            min( max( int( (pt(1) - gl_yl) * inv_resolution), 0), GLY_SIZE - 1),
            min( max( int( (pt(2) - gl_zl) * inv_resolution), 0), GLZ_SIZE - 1);                    
    return idx;
}
Eigen::Vector3d AstarPathFinder::coordRounding(const Eigen::Vector3d & coord)
{
    return gridIndex2coord(coord2gridIndex(coord));
}
inline bool AstarPathFinder::isOccupied(const Eigen::Vector3i & index) const
{
    return isOccupied(index(0), index(1), index(2));
}
inline bool AstarPathFinder::isFree(const Eigen::Vector3i & index) const
{
    return isFree(index(0), index(1), index(2));
}
inline bool AstarPathFinder::isOccupied(const int & idx_x, const int & idx_y, const int & idx_z) const 
{
    return  (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE && 
            (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] == 1));
}
inline bool AstarPathFinder::isFree(const int & idx_x, const int & idx_y, const int & idx_z) const 
{
    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE && 
           (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] < 1));
}
//jieming
int isInClosed(GridNodePtr *** GridNodeMap, const Eigen::Vector3i& temp_coord) 
{
    int flag = 0;
    //cout<<temp_coord<<endl;
    try{
        if(GridNodeMap[temp_coord(0)][temp_coord(1)][temp_coord(2)]->id == -1){
            flag = 1;
        }
    }
    catch(...){
        cout<< temp_coord(0) << temp_coord(1)<< temp_coord(2)<<endl;
    }
    return flag;
}
//int isInOpen(const std::multimap<double, GridNodePtr>& openSet, const Eigen::Vector3i& temp_coord) 
int isInOpen(GridNodePtr *** GridNodeMap, const Eigen::Vector3i& temp_coord) 
{
    int flag = 0;
    //if(GridNodeMap[temp_coord(0)][temp_coord(1)][temp_coord(2)]->gScore < LONG_MAX){
    if(GridNodeMap[temp_coord(0)][temp_coord(1)][temp_coord(2)]->id ==1){
        flag = 1;
    }
    return flag;  
}
void AstarPathFinder::AstarGetSucc(GridNodePtr currentPtr, vector<GridNodePtr> & neighborPtrSets, vector<double> & edgeCostSets)
{   
    neighborPtrSets.clear();
    edgeCostSets.clear();
    Eigen::Matrix<int,26,3> directions;
    directions << -1,0,0,  0,1,0,  1,0,0,  0,-1,0, 0,0,1, 0,0,-1,  //1
                  -1,1,0,  1,1,0, 1,-1,0, -1,-1,0, //sqrt(2)
                  -1,0,1,  0,1,1,  1,0,1,  0,-1,1, //sqrt(2)
                 -1,0,-1, 0,1,-1, 1,0,-1, 0,-1,-1, //sqrt(2)
                  -1,1,1,  1,1,1, 1,-1,1, -1,-1,1, //sqrt(3)
                 -1,1,-1, 1,1,-1, 1,-1,-1, -1,-1,-1;//sqrt(3)

    for(int row=0; row<directions.rows(); row++)
    {   
        Eigen::Vector3i temp_coord = Eigen::Vector3i(directions.row(row)) + currentPtr->index;
        // if out of map then continue
        if(temp_coord(0)<0 || temp_coord(0)>=GLX_SIZE || temp_coord(1)<0 || temp_coord(1)>=GLY_SIZE || temp_coord(2)<0 || temp_coord(2)>=GLZ_SIZE){
            continue;
        }
        else if(data[temp_coord(0)*GLYZ_SIZE+temp_coord(1)*GLZ_SIZE+temp_coord(2)]==1){
            continue;
        }
        // if in closed list then continue
        else if(isInClosed(GridNodeMap, temp_coord)){
            //cout<<"in closed!!!!"<<endl;
            continue;
        }
        else{
            if(isInOpen(GridNodeMap,temp_coord)){
                //GridNodeMap[temp_coord(0)][temp_coord(1)][temp_coord(2)]->id = 1;
            }
            else{ // new point
                //GridNodeMap[temp_coord(0)][temp_coord(1)][temp_coord(2)]->id = 0;
            }
            // set neighbor
            if(row<=5){
                edgeCostSets.push_back(1 + currentPtr->gScore);
                //GridNodeMap[temp_coord(0)][temp_coord(1)][temp_coord(2)]->gScore = 1 + currentPtr->gScore;

            }
            else if(row<=17 && row>5){
                edgeCostSets.push_back(sqrt(2)* + currentPtr->gScore);
                //GridNodeMap[temp_coord(0)][temp_coord(1)][temp_coord(2)]->gScore =sqrt(2)* + currentPtr->gScore;


            }
            else{
                edgeCostSets.push_back(sqrt(3) + currentPtr->gScore);
                //GridNodeMap[temp_coord(0)][temp_coord(1)][temp_coord(2)]->gScore =sqrt(3)+ currentPtr->gScore;
            }

            GridNodeMap[temp_coord(0)][temp_coord(1)][temp_coord(2)]->dir   = Eigen::Vector3i(directions.row(row));
            //GridNodeMap[temp_coord(0)][temp_coord(1)][temp_coord(2)]->index = temp_coord;
            //GridNodeMap[temp_coord(0)][temp_coord(1)][temp_coord(2)]->coord = gridIndex2coord(temp_coord);
            GridNodeMap[temp_coord(0)][temp_coord(1)][temp_coord(2)]->cameFrom =  currentPtr;

            neighborPtrSets.push_back(GridNodeMap[temp_coord(0)][temp_coord(1)][temp_coord(2)]);
        }
    }
}

double AstarPathFinder::getHeu(GridNodePtr node1, GridNodePtr node2)
{
    Eigen::Vector3i dist = node1->index - node2->index;
    
    double hScore = dist.norm()*(1 + 1/(50*50*25));                   
    //double hScore = dist.array().abs().sum();      Manhatten distance

    // diagonal distance
    /*
    dist = dist.cwiseAbs();
    int dmin = dist.minCoeff();
    int dmax = dist.maxCoeff();
    double dmid = dist(0) + dist(1) + dist(2) - dmax -dmin;
    double hScore=(1.7-1.4)*dmin + (1.4-1)*dmid + 1*dmin;
    */

    double fScore= hScore + node1->gScore;
    return fScore;
}

void AstarPathFinder::AstarGraphSearch(Vector3d start_pt, Vector3d end_pt)
{   
    ros::Time time_1 = ros::Time::now();
    Vector3i start_idx = coord2gridIndex(start_pt);
    Vector3i end_idx   = coord2gridIndex(end_pt);
    goalIdx = end_idx;
    start_pt = gridIndex2coord(start_idx);
    end_pt   = gridIndex2coord(end_idx);
    GridNodePtr startPtr = new GridNode(start_idx, start_pt);
    GridNodePtr endPtr   = new GridNode(end_idx,   end_pt);
    openSet.clear();
    GridNodePtr currentPtr  = NULL;
    GridNodePtr neighborPtr = NULL;
    startPtr -> gScore = 0;
    startPtr -> fScore = getHeu(startPtr,endPtr);
    startPtr -> id = 1;
    startPtr -> coord = start_pt;
    openSet.insert( make_pair(startPtr -> fScore, startPtr) );

    vector<GridNodePtr> neighborPtrSets;
    vector<double> edgeCostSets;
    //step 2:assignNodesInf
    for(int i = 0; i < GLX_SIZE; i++)
        for(int j = 0; j < GLY_SIZE; j++)
            for(int k = 0; k < GLZ_SIZE; k++){
                if(i==start_idx(0) && j==start_idx(1) && k==start_idx(2)){
                    GridNodeMap[i][j][k]->gScore = 0;
                }
                else{
                    GridNodeMap[i][j][k]->gScore = LONG_MAX;
                }
                GridNodeMap[i][j][k]->id = 0;
            }
    while ( !openSet.empty() ){
        // step 3: Remove the node with lowest cost function from open set to closed set
        currentPtr = openSet.begin()->second;
        currentPtr->id=-1;       
        openSet.erase(openSet.begin());

        if( currentPtr->index == goalIdx ){
            ros::Time time_2 = ros::Time::now();
            terminatePtr = currentPtr;
            ROS_WARN("[A*]{sucess}  Time in A*  is %f ms, path cost if %f m", (time_2 - time_1).toSec() * 1000.0, currentPtr->gScore * resolution );            
            delete startPtr;
            delete endPtr;
            return;
        }

        AstarGetSucc(currentPtr, neighborPtrSets, edgeCostSets);  //STEP 4: finish AstarPathFinder::AstarGetSucc yourself
        for(int i = 0; i < (int)neighborPtrSets.size(); i++){
            double newGscore = edgeCostSets[i];
            neighborPtr = neighborPtrSets[i];
            neighborPtr->fScore = getHeu(neighborPtr, endPtr);

            if(neighborPtr -> id == 0){ //discover a new node, which is not in the closed set and open set
                neighborPtr->id = 1;
                neighborPtr->gScore = newGscore;
                neighborPtr->nodeMapIt = openSet.insert(make_pair(neighborPtr->fScore, neighborPtr));
                continue;
            }
            else if(neighborPtr-> id == 1){ //this node is in open set and need to judge if it needs to update, the "0" should be deleted when you are coding
                //cout<<"already in open"<<endl;
                if(newGscore < neighborPtr->gScore){
                    auto it = neighborPtr->nodeMapIt;
                    openSet.erase(it);
                    neighborPtr->gScore = newGscore;
                    neighborPtr->nodeMapIt = openSet.insert(make_pair(neighborPtr->fScore, neighborPtr));
                }
                continue;


                /*
                                for(auto item:openSet){
                    if(item.second->index == neighborPtr->index && item.second->gScore > neighborPtr->gScore){
                        openSet.erase(item.first);
                        openSet.insert(make_pair(neighborPtr->fScore, neighborPtr));
                        //cout << "update open!!!!!!!!" <<endl;
                        break;
                    }
                }

                */
            }
            else{//this node is in closed set
                continue;
            }
        }
         
    }
    //if search fails
    ros::Time time_2 = ros::Time::now();
    if((time_2 - time_1).toSec() > 0.1)
        ROS_WARN("Time consume in Astar path finding is %f", (time_2 - time_1).toSec() );
    cout<<"fail"<<endl;
    delete startPtr;
    delete endPtr;
}


vector<Vector3d> AstarPathFinder::getPath() 
{   
    vector<Vector3d> path;
    vector<GridNodePtr> gridPath;

    GridNodePtr parent = terminatePtr->cameFrom;
    while(parent->cameFrom != NULL){
        gridPath.push_back(parent);
        parent = parent->cameFrom;
    }
    for (auto ptr: gridPath)
        path.push_back(ptr->coord);
        
    reverse(path.begin(),path.end());


    return path;
}

        //cout<< openSet.begin()->second->id  <<endl;
        //cout<< currentPtr->id  <<endl;

        //         std::multimap<double, GridNodePtr>::iterator  i, iend;
        // iend = openSet.end();
        //cout<< "start"<<endl;
        // for (i=openSet.begin(); i!=iend; ++i){
            //cout << (*i).second << '    ' << (*i).first <<endl;
            //cout << (*i).second->gScore <<endl;
        // }