#include "planner/RRTstar.h"

using namespace std;
using namespace octomap;

RRTstar::RRTstar(OcTree* map, double* start_pose, double* goal_pose) {
    this->map = map;
    this->start_pose = start_pose;
    this->goal_pose = goal_pose;
    srand(time(NULL));
};

RRTstar::~RRTstar() {

}

void RRTstar::correctIdx(int &x, int &y, int &z) {
    x = abs(x) + max_x - 1;
    y = abs(y) + max_y - 1;
    z = abs(z) + max_z - 1;
}

int RRTstar::GetMapIndex(int x, int y, int z) {
    // assuming 0 start indexing
    int index = z * x_size * y_size;
    index = index + (y * x_size) + x;
    return index;
}

int RRTstar::IsValidPose(double* pose) {
    // determine if pose is valid in map
    // check map boundaries
    if (pose[0] < min_x || pose[0] >= x_size ||
        pose[1] < min_y || pose[1] >= y_size ||
        pose[2] < min_z || pose[2] >= z_size) {
        return 0;
    }

    OcTreeNode* node = map->search(int(pose[0]), int(pose[1]), int(pose[2]), 0);
    int occupied = 0;

    // if (node == 0) // if the node does not exist, assume it is unoccupied
    // {
    //     return 0;
    // }

    // Check within 0.5 metres to make sure the drone has enough space to fly
    for (OcTree::leaf_bbx_iterator it = map->begin_leafs_bbx(point3d (int(pose[0])-clearance, int(pose[1])-clearance, int(pose[2])-clearance), 
            point3d (int(pose[0])+clearance, int(pose[1])+clearance, int(pose[2])+clearance)), end = map->end_leafs_bbx(); it != end; it++)
    {
        occupied = map->isNodeOccupied(*it);
        if (occupied == 1)
        {
            numoccupied +=1;
            return 0;
        }
    }
    return 1;
}

int RRTstar::ClearPath(double* pose1, double* pose2) {
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

RRTstar::Q* RRTstar::Sample() {
    numsampled += 1;
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
        qsample->pose[i] = min_sample[i] + (rand() % (max_sample[i] - min_sample[i]));
    }
    // while (!IsValidPose(qsample->pose)) {
    //     qsample = Sample();
    // }
    return qsample;
}

int RRTstar::Same(Q* q1, Q* q2) {
    // check if q1 and q2 are at the same pose
    for (int i=0; i<numofDOFs; i++) {
        if (q1->pose[i] != q2->pose[i]) {
            return 0;
        }
    }
    return 1;
}

RRTstar::Q* RRTstar::NearestNeighbor(Q* q) {
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

double RRTstar::EuclideanDist(double* q1_pose, double* q2_pose) {
    double dist = 0.0;
    for (int i=0; i<numofDOFs-1; i++) {
        dist += pow((q1_pose[i] - q2_pose[i]), 2);
    }
    dist = sqrt(dist);
    return dist;
}

int RRTstar::MoveUntilObstacle(Q* qnearest, Q* qsample, Q* qnew) {
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

double RRTstar::Cost(Q* q1, Q* q2) {
    return EuclideanDist(q1->pose, q2->pose);
}

double RRTstar::GetRadius(){
    double gamma = 100;
    double n = V.size();
    return double(gamma * pow((log(n)/n),(1/double(numofDOFs))));
}

RRTstar::Q* RRTstar::Steer(Q* qnearest, Q* qsample) {
    // move eps distance from qnearest toward qsample
    Q* qnew = new Q;
    double temp[numofDOFs];
    double dist[numofDOFs];
    float maxdist = 0.0;
    for (int i = 0; i < numofDOFs - 1; i ++) {
        dist[i] = qsample->pose[i] - qnearest->pose[i];
        if (fabs(dist[i]) > eps) {
            dist[i] = (dist[i] > 0? eps : (-1.0*eps));
        } 
        if (fabs(dist[i]) > maxdist) {
            maxdist = fabs(dist[i]);
        }
    }
    float steps = 30.0;
    for (int step=1; step<int(steps+1); step++) {
        for (int i=0; i<numofDOFs; i++) {
            temp[i] = qnearest->pose[i] + step*(dist[i]/steps);
        }
        if (IsValidPose(temp)) {
            for (int i=0; i<numofDOFs; i++) {
                qnew->pose[i] = temp[i];
            }
        }
        else {
            if (step > 1) {
                return qnew;
            }
            else {
                return nullptr;
            }
        }
    }
    return qnew;
}

vector<RRTstar::Q*> RRTstar::Near(Q* qnew, double rad) {
    // look through V and get all qnear within radius rad of qnew
    vector<Q*> Qnear;
    for (auto qnear : V) {
        float dist = MaxDist(qnew, qnear);
        if (dist < rad) {
            Qnear.push_back(qnear);
        }
    }
    return Qnear;
}

void RRTstar::Extend(Q* q) {
    // Extend map toward q
    double radius = min(GetRadius(), eps);
    auto qnearest = NearestNeighbor(q);
    auto qnew = Steer(qnearest, q);
    if (qnew != nullptr) {
        qnew->parent = qnearest;
        V.push_back(qnew);
        qnew->cost = qnearest->cost + Cost(qnearest, qnew);
        // Rewire
        Q* qmin = new Q;
        qmin = qnearest;
        auto Qnear = Near(qnew, radius);
        for (auto qnear : Qnear) {
            float cost = qnear->cost + Cost(qnear, qnew);
            if (cost < qnew->cost) {
                qmin = qnear;
            }
        }
        qnew->parent = qmin;
        for (auto qnear : Qnear) {
            if (qnear == qmin) { continue; }
            else if (ClearPath(qnew->pose,qnear->pose) && (qnear->cost > (qnew->cost + Cost(qnew,qnear)))) {
                qnear->parent = qnew;
            }
        }
    }
    else{
        auto qsample = Sample();
        Extend(qsample);
    }
}

stack<RRTstar::Q*> RRTstar::Interpolate(Q* qnear, Q* qfar) { //near is goalside
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

double RRTstar::MaxDist(Q* q1, Q* q2) {
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

void RRTstar::BuildRRT(Q* qgoal) {
    time(&start);
    time(&end);
    int goalfound = 0;
    while(((end-start) <= float(runtime)) && !interrupt) {
        auto qsample = Sample();
        Extend(qsample);
        auto q = FindAnytimeGoal(qgoal);
        if (q != nullptr) {
            Interrupt();
            return;
        }
        time(&end);
    }
}

RRTstar::Q* RRTstar::FindBestNearGoalQ(Q* qgoal) {
    // search V to see if goal pose has been found
    Q* qmin = new Q;
    int found = 0;
    for (auto q : V) {
        int nextq = 0;
        for (int i = 0; i < numofDOFs - 1; i++) {
            if (abs((q->pose[i] - qgoal->pose[i])) > 3) { // search within max 3 map grids from goal
                nextq = 1;
                break;
            }
        }
        if (!nextq && ClearPath(q->pose, qgoal->pose)) {
            if ((q->cost + Cost(q,qgoal)) < bestcost) {
                bestcost = q->cost + Cost(q,qgoal);
                qmin = q;
                found++;
            }
        }
    }
    if (found == 0) return nullptr; 
    return qmin;
}

RRTstar::Q* RRTstar::FindAnytimeGoal(Q* qgoal) {
    // search V to see if goal pose has been found
    auto q = V.back();
    for (int i = 0; i < numofDOFs - 1; i++) {
        if (abs((q->pose[i]) - qgoal->pose[i]) > 3.0) { // search within max 3 map grids from goal
            return nullptr;
        }
    }
    if (ClearPath(q->pose, qgoal->pose)) {
        bestcost = q->cost + Cost(q,qgoal);
    }
    return q;
}

vector<vector<int>> RRTstar::MakePlan(Q* qstart, Q* qgoal) {
    cout <<"qstart "<<qstart<<endl;
    cout<<"qgoal "<<qgoal<<endl;
    // trace parents back from qgoal to qstart
    stack<Q*> planstack;
    planstack.push(qgoal);
    while (planstack.top() != qstart) {
        cout <<"planstack top " << planstack.top() << endl;
        // if (MaxDist(planstack.top(), planstack.top()->parent) > 4) {
        //     auto interstack = Interpolate(planstack.top(), planstack.top()->parent);
        //     while (!interstack.empty()) {
        //         planstack.push(interstack.top());
        //         interstack.pop();
        //     }
        // }
        // else {
        planstack.push(planstack.top()->parent);
        // }
    }

    vector<vector<int>> plan;
    while (!planstack.empty()) {
        vector<int> temp;
        for (int i = 0; i < numofDOFs; i ++) {
            temp.push_back(planstack.top()->pose[i]);
        }
        plan.push_back(temp);
        planstack.pop();
    }
    planlength = plan.size();
    return plan;
}

vector<vector<int>> RRTstar::RunRRT() {
    Q* qgoal = new Q;
    for (int i = 0; i < numofDOFs; i++) {
        qgoal->pose[i] = goal_pose[i];
    }

    Q* qstart = new Q;
    qstart->cost = 0.0;
    for (int i = 0; i < numofDOFs; i ++) {
        qstart->pose[i] = start_pose[i];
    }
    qstart->parent = qstart;
    V.push_back(qstart);
    vector<vector<int>> plan;
    if (!IsValidPose(qgoal->pose)) {
        cout << "Invalid Goal Pose" << endl;
        return plan;
    }
    BuildRRT(qgoal);
    auto qneargoal = FindAnytimeGoal(qgoal);
    // auto qneargoal = FindBestNearGoalQ(qgoal);
    if (qneargoal != nullptr) {
        cout << "qneargoal " << qneargoal->pose[0]<<", "<<qneargoal->pose[1]<<", "<<qneargoal->pose[2]<<endl;
        cout <<"qneargoal parent " << qneargoal->parent <<endl;
        cout << "qgoal " << qgoal->pose[0]<<", "<<qgoal->pose[1]<<", "<<qgoal->pose[2]<<endl;
        cout <<"qgoal parent " << qgoal->parent <<endl;
        qgoal->parent = qneargoal;
        plan = MakePlan(qstart, qgoal);
    }
    cout  << "V size: " << V.size() << endl;
    cout << "Plan Cost: " << bestcost << endl;
    return plan;
}

void RRTstar::Run() {
    g_plan = RunRRT();
    return;
}

vector<vector<int>> RRTstar::getGPlan() {
    return g_plan;
}

int RRTstar::getPlanlength() {
    return this->planlength;
}

double RRTstar::getPlanCost() {
    return bestcost;
}

vector<vector<int>> RRTstar::getGraph() {
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

void RRTstar::Interrupt() {
    this->interrupt = 1;
}
 
int RRTstar::getGraphSize() {
    return V.size();
}