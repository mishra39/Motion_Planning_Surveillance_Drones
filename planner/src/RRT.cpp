#include "planner/RRT.h"

using namespace std;
using namespace octomap;

RRT::RRT(octomap::OcTree* map, double* start_pose, double* goal_pose) {
    this->map = map;
    this->start_pose = start_pose;
    this->goal_pose = goal_pose;

    min_sample[0] = min_x;
    min_sample[1] = min_y;
    min_sample[2] = min_z;
    min_sample[3] = 0;

    max_sample[0] = max_x;
    max_sample[1] = max_y;
    max_sample[2] = max_z;
    max_sample[3] = 180;
};

RRT::~RRT() {

}

void RRT::correctIdx(int &x, int &y, int &z) {
    x = abs(x) + max_x - 1;
    y = abs(y) + max_y - 1;
    z = abs(z) + max_z - 1;
}

int RRT::GetMapIndex(int x, int y, int z) {
    // assuming 0 start indexing
    int index = z * x_size * y_size;
    index = index + (y * x_size) + x;
    return index;
}

int RRT::IsValidPose(double* pose) {
    // determine if pose is valid in map
    // check map boundaries
    if (pose[0] < min_x || pose[0] >= x_size ||
        pose[1] < min_y || pose[0] >= y_size ||
        pose[2] < min_z || pose[2] >= z_size) {
        return 0;
    }
    
    OcTreeNode* node = map->search(int(pose[0]), int(pose[1]), int(pose[2]), 0);
    int occupied = 0;

    if (node == 0) // if the node does not exist, it is not valid
    {
        return 0;
    }
        // Check within 0.8 metres to make sure the drone has enough space to fly
    for (OcTree::leaf_bbx_iterator it = map->begin_leafs_bbx(point3d (int(pose[0])-clearance, int(pose[1])-clearance, int(pose[2])-clearance), 
            point3d (int(pose[0])+clearance, int(pose[1])+clearance, int(pose[2])+clearance)), end = map->end_leafs_bbx(); it != end; it++)
    {
        occupied = map->isNodeOccupied(*it);
        if (occupied == 0)
        {
            return 1;
        }
    }
    return 0;
}

int RRT::ClearPath(double* pose1, double* pose2) {
    // check if path is clear from pose1 to pose2
    double dist[numofDOFs-1];
    double maxdist = 0;
    for (int i = 0; i < numofDOFs - 1; i ++) {
        dist[i] = pose2[i] - pose1[i];
        if (abs(dist[i]) > maxdist) {
            maxdist = abs(dist[i]);
        }
    }
    double temp[numofDOFs];
    for (int j = 0; j < maxdist; j++) {
        for (int k = 0; k < numofDOFs - 1; k ++) {
            temp[k] = pose1[k] + int(j * (dist[k] / maxdist));
        }
        if (!IsValidPose(temp)) {
            return 0;
        }
    }
    return 1;
}

RRT::Q* RRT::Sample() {
    // generate random pose
    Q* qsample = new Q;
    // use goal biasing
    if (goalBias) {
        uniform_real_distribution<double> probability(0,1);
        float prob = (double) (rand() / double(RAND_MAX));
        if (prob < goal_bias_value) {
            for (int i = 0; i < numofDOFs; i++) {
                qsample->pose[i] = goal_pose[i];
            }  
            return qsample;  
        }
    }

    // otherwise
    for (int i = 0; i < numofDOFs; i++) {
        // uniform_real_distribution<double> angle(0, 2*PI);
        // qrand.angles[j] = angle(random);
        qsample->pose[i] = rand() % sizes[i];
    }
    while (!IsValidPose(qsample->pose)) {
        qsample = Sample();
    }
    return qsample;
}

int RRT::Same(Q* q1, Q* q2) {
    // check if q1 and q2 are at the same pose
    for (int i=0; i<numofDOFs; i++) {
        if (q1->pose[i] != q2->pose[i]) {
            return 0;
        }
    }
    return 1;
}

RRT::Q* RRT::NearestNeighbor(Q* q) {
    // Find Nearest Vertex Neighbor
    Q* qnearest = new Q;
    double min_dist = INFINITY;
    for (auto qnear : V) {
        float dist = EuclideanDist(q->pose, qnear->pose);
        if (dist < min_dist) {
            min_dist = dist;
            qnearest = qnear;
        }
    }
    return qnearest;
}

double RRT::EuclideanDist(double* q1_pose, double* q2_pose) {
    double dist = 0.0;
    for (int i=0; i<numofDOFs; i++) {
        dist += pow((q1_pose[i] - q2_pose[i]), 2);
    }
    dist = sqrt(dist);
    return dist;
}

int RRT::MoveUntilObstacle(Q* qnearest, Q* qsample, Q* qnew) {
    // move in straight line from qnearest toward qsample until obstacle or eps distance
    double maxdist = 0.0;
    double dist[numofDOFs];
    for (int i = 0; i < (numofDOFs - 1); i++) {
        dist[i] = qsample->pose[i] - qnearest->pose[i];
        if (fabs(dist[i]) > eps) {
            dist[i] = ((dist[i] > 0)? eps : (-1.0*eps));
        }
        if (fabs(dist[i]) > maxdist) {
            maxdist = fabs(dist[i]);
        }
    }
    dist[theta] = qsample->pose[theta] - qnearest->pose[theta];

    double steps = 50;
    double temp[numofDOFs];
    for (int step = 1; step < int(steps+1); step++) {
        for (int i = 0; i < numofDOFs; i++) {
            temp[i] = qnearest->pose[i] + step * (dist[i]/steps);
        }
        if (IsValidPose(temp)) {
            for (int i = 0; i < numofDOFs; i++) {
                qnew->pose[i] = temp[i];
            }
        }
        else {
            return step;
        }
    }
    return steps;
}

void RRT::Extend(Q* q) {
    // Extend map towards q
    // Move in direction of q from q parent
    auto qnearest = NearestNeighbor(q);
    Q* qnew = new Q;
    if (MoveUntilObstacle(qnearest, q, qnew) > 1) {
        qnew->parent = qnearest;
        qnew->cost = qnearest->cost + Cost(qnearest,qnew);
        V.push_back(qnew);
    }
}

double RRT::Cost(Q* q1, Q* q2) {
    return EuclideanDist(q1->pose, q2->pose);
}

RRT::Q* RRT::ReachedGoal(Q* qgoal) {
    auto q = V.back();
    for (int i = 0; i < (numofDOFs-1); i++) {
        if ((q->pose[i] - qgoal->pose[i]) > 5) {
            return nullptr;
        }
    }
    double dist = q->pose[theta] - qgoal->pose[theta];
    if (dist < tol) {
        return q;
    }
    return nullptr;
}

stack<RRT::Q*> RRT::Interpolate(Q* qnear, Q* qfar) { //near is goalside
    // create a stack of points to interpolate far away pts
    stack<Q*> interstack; //base: qfar, top: qnear (already in pathstack)
    float maxdist = 0.0;
    for (int i=0; i<(numofDOFs-1); i++) {
        float tempdist = qfar->pose[i] - qnear->pose[i];
        if (fabs(tempdist) > maxdist) {
            maxdist = fabs(tempdist);
        }
    }
    float steps = maxdist / 4;
    double dist[numofDOFs];
    interstack.push(qfar);
    for (int i=0; i<numofDOFs; i++) {
        dist[i] = (qnear->pose[i] - qfar->pose[i])/steps;
    }
    for (int i=1; i<int(steps); i++) {
        Q* qnew = new Q;
        for (int j=0; j<numofDOFs; j++) {
            qnew->pose[j] = qfar->pose[j] + int(i*dist[j]);
        }
        interstack.push(qnew);
    }
    return interstack;
}

double RRT::MaxDist(Q* q1, Q* q2) {
    // get max distance between q1 and q2
    double maxdist = 0.0;
    for (int i = 0; i < (numofDOFs-1); i++) {
        double dist = fabs(q1->pose[i] - q2->pose[i]);
        if (dist > maxdist) {
            maxdist = dist;
        }
    }
    return maxdist;
}

vector<vector<int>> RRT::MakePlan(Q* qstart, Q* qgoal) {
    // make a stack of the Q's in the path
    plancost = qgoal->cost;
    stack<Q*> pathstack;
    pathstack.push(qgoal);
    while (pathstack.top() != qstart) {
        if (MaxDist(pathstack.top(), pathstack.top()->parent) > 4) {
            Q* qparent = pathstack.top()->parent;
            auto interstack = Interpolate(pathstack.top(), pathstack.top()->parent);
            while (!interstack.empty()) {
                pathstack.push(interstack.top());
                interstack.pop();
            }
        }
        else {
            pathstack.push(pathstack.top()->parent);
        }
    }
    planlength = pathstack.size();

    // put them into "path"
    // put in vector for local biasing
    vector<vector<int>> plan;
    int j = 0;
    while (!pathstack.empty()) {
        vector<int> temp;
        for (int i = 0; i < numofDOFs; i ++) {
            temp.push_back(pathstack.top()->pose[i]);
        }
        plan.push_back(temp);
        pathstack.pop();
    }
    return plan;
}

void RRT::BuildRRT() {
    time(&start);
    time(&end);
    while(((end-start) <= run_time) && !interrupt) {
        auto qsample = Sample();
        Extend(qsample);
        time(&end);
    }
}

vector<vector<int>> RRT::RunRRT() {
    Q* qgoal = new Q;
    for (int i = 0; i < numofDOFs; i++) {
        qgoal->pose[i] = goal_pose[i];
    }

    Q* qstart = new Q;
    for (int i = 0; i < numofDOFs; i++) {
        qstart->pose[i] = start_pose[i];
    }
    qstart->parent = qstart;
    qstart->cost = 0.0;
    V.push_back(qstart);
    int count = 0;
    time(&start);
    time(&end);

    vector<vector<int>> plan;
    BuildRRT();
    auto qneargoal = ReachedGoal(qgoal);
    if (qneargoal != nullptr) {
        if ((qneargoal->cost + Cost(qneargoal,qgoal)) < bestcost) {
            qgoal->parent = qneargoal;
            qgoal->cost = qneargoal->cost + Cost(qneargoal, qgoal);
            plan = MakePlan(qstart, qgoal);
        }
    }
    // Returns first plan found
    return plan;
}

void RRT::Run() {
    g_plan = RunRRT();
    return;
}

vector<vector<int>> RRT::getGPlan() {
    return g_plan;
}

int RRT::getPlanlength() {
    return this->planlength;
}

double RRT::getPlanCost() {
    return plancost;
}

vector<vector<int>> RRT::getGraph() {
    vector<vector<int>> graph;
    for (auto q : V) {
        vector<int> temp;
        for (int i = 0; i < numofDOFs; i ++) {
            temp.push_back(q->pose[i]);
        }
        graph.push_back(temp);
        vector<int> parent;
        for (int i = 0; i < numofDOFs; i ++) {
            parent.push_back(q->parent->pose[i]);
        }
        graph.push_back(parent);
    }
    return graph;
}

void RRT::Interrupt() {
    this->interrupt = 1;
}