#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);

}


float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
  return node->distance(*end_node);

}


void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
  current_node->FindNeighbors();
  auto neighbors = current_node->neighbors;
  for(int i=0; i < neighbors.size() ; i++) {
    neighbors[i]->parent = current_node;
    neighbors[i]->h_value = CalculateHValue(neighbors[i]);
    neighbors[i]->g_value =  current_node->g_value + neighbors[i]->distance(*current_node);
    open_list.push_back(neighbors[i]);
    neighbors[i]->visited = true;    
  }
}


RouteModel::Node *RoutePlanner::NextNode() {
  std::sort (open_list.begin(), open_list.end(), [](RouteModel::Node* node1, RouteModel::Node* node2) 
             { 
               return (node1->h_value + node1->g_value) > (node2->h_value + node2->g_value);
             }) ;
  RouteModel::Node* lowest_sum_node = open_list.back() ;
  open_list.pop_back();
  return lowest_sum_node ;
}



std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
  
    path_found.push_back(*current_node);
    while(current_node->parent != nullptr){
      path_found.push_back(*(current_node->parent));
      distance = distance + current_node->distance(*(current_node->parent));
      current_node = current_node->parent ;     
    }
  
    std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;
  
    // TODO: Implement your solution here.
    open_list.push_back(start_node);
    start_node->visited = true;
    AddNeighbors(start_node);
    

  while(open_list.size() > 0){
     current_node = NextNode();
     if(current_node->distance(*end_node) == 0 ){
     m_Model.path = ConstructFinalPath(current_node);
       return;
     }
     AddNeighbors(current_node);
  }

}
