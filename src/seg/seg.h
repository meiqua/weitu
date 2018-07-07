#ifndef SEG_H
#define SEG_H

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include "CIEDE2000/CIEDE2000.h"

namespace seg_helper {

namespace min_span_tree {
struct Vertex{
    int id;
};
struct Edge {
    double weight;
    Vertex v1;
    Vertex v2;
    bool operator<(const Edge& rhs) const{
      return weight < rhs.weight;
    }
};
struct DisjointSets {
    std::vector<int> parent, rnk;
    DisjointSets(int n){
        parent.resize(n);
        rnk.resize(n, 0);
        for(int i=0; i<parent.size();i++){
            parent[i] = i;
        }
    }
    int find(int u){
        while(u != parent[u]){
            u = parent[u];
        }
        return u;
    }
    void merge(int x, int y){
        x = find(x);
        y = find(y);
        if(rnk[x] > rnk[y]){
            parent[y] = x;
        }else{
            parent[x] = y;
        }
        if(rnk[x]==rnk[y]){
            rnk[y]++;
        }
    }
};
struct Graph {
    int V_size;
    std::vector<Edge> edges, mst_edges;
    double mst_cost;

    void add_edge(Vertex& v1, Vertex& v2, cv::Mat& lab){

        int cols = lab.cols;
        auto c1 = lab.at<char>(v1.id/cols,v1.id%cols);
        auto c2 = lab.at<char>(v2.id/cols,v2.id%cols);

        double weight = std::abs(double(c1-c2))/255.0*100.0;

        Edge edge;
        edge.v1 = v1;
        edge.v2 = v2;
        edge.weight = weight;
        edges.push_back(edge);
    }
    double kruskalMST(){
        double cost = 0;

        std::sort(edges.begin(), edges.end());

        DisjointSets ds(V_size);
        int counts = 0;
        for(auto& edge: edges){
            int v_id1 = edge.v1.id;
            int v_id2 = edge.v2.id;
            int v_parent1 = ds.find(v_id1);
            int v_parent2 = ds.find(v_id2);
            if(v_parent1 != v_parent2){
                ds.merge(v_parent1, v_parent2);
                cost += edge.weight;
                mst_edges.push_back(edge);
                counts++;
                if(counts==V_size-1){
                    break;
                }
            }
        }
        return cost;
    }
    Graph(cv::Mat& lab){
        V_size = lab.rows*lab.cols;
        for(int i=1; i<lab.rows-2; i++)  // 1 padding
            for(int j=1; j<lab.cols-2; j++){
                Vertex v0, v1, v2, v3, v4, v5, v6, v7, v8;
                v0.id = j + i*lab.cols;
                v1.id = j+1+i*lab.cols;
                v2.id = j+(i+1)*lab.cols;
                v3.id = j+1+(i+1)*lab.cols;

//                v4.id = j+(i+2)*lab.cols;
//                v5.id = j+2+(i)*lab.cols;
//                v6.id = j+1+(i+2)*lab.cols;
//                v7.id = j+2+(i+1)*lab.cols;
//                v8.id = j+2+(i+2)*lab.cols;

                add_edge(v0, v1, lab);
                add_edge(v0, v2, lab);
                add_edge(v0, v3, lab);
                add_edge(v1, v2, lab);

//                add_edge(v0, v4, lab);
//                add_edge(v0, v5, lab);
//                add_edge(v0, v6, lab);
//                add_edge(v0, v7, lab);
//                add_edge(v0, v8, lab);
            }
        mst_cost = kruskalMST();
    }
};

}


}

class Segmentation {
public:
    Segmentation(cv::Mat lab,
                 std::vector<seg_helper::min_span_tree::Edge>& edges){
        for(int i=0; i<lab.rows; i++)
            for(int j=0; j<lab.cols; j++){
                entry e1;
                e1.id = j + i*lab.cols;
                e1.parent = e1.id;
                V_list.push_back(e1);
                assert(e1.id == V_list.size()-1);
            }
        this->edges = std::move(edges);
        level_recorder.resize(1); //we don't want level 0, just empty
    }
    struct entry{
        int id;
        int parent;
        int count = 1;
        int level=0;
        double merging_cost=0;
    };
    std::vector<entry> V_list;
    std::vector<seg_helper::min_span_tree::Edge> edges;
    std::vector<std::vector<entry>> level_recorder;

    void merge(seg_helper::min_span_tree::Edge& edge);
    int find(entry& u, std::vector<entry>& V_list);
    std::vector<std::vector<std::vector<int>>> process();
};
#endif
