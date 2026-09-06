#include <cmath>
#include <iostream>
#include <stdexcept>
#include <grid_map_core/GridMap.hpp>
#include "groundgrid/PlanningGrid.h"

// This comparison uses the installed ROS distribution's actual grid_map_core.
// The dependency-free test alone cannot certify parity with a vendor installation.
int main() {
    try {
        for(double resolution:{0.10,0.15,0.30}) for(double cx:{0.0,1.237,-17.331}) {
            grid_map::GridMap map;
            map.setGeometry(grid_map::Length(12.0,9.0),resolution,grid_map::Position(cx,-3.271));
            for(int row:{0,1,17}) for(int col:{0,1,23}) {
                map.setStartIndex(grid_map::Index(row,col));
                groundgrid::PlanningGrid plain;
                plain.rows=map.getSize()(0); plain.cols=map.getSize()(1);
                plain.start_row=row; plain.start_col=col;
                plain.center_x=map.getPosition().x(); plain.center_y=map.getPosition().y();
                plain.length_x=map.getLength().x(); plain.length_y=map.getLength().y();
                plain.resolution=resolution;
                for(int r=0;r<plain.rows;++r) for(int c=0;c<plain.cols;++c) {
                    grid_map::Position actual; groundgrid::PlanningPosition expected;
                    map.getPosition(grid_map::Index(r,c),actual); plain.getPosition({r,c},expected);
                    if(actual.x()!=expected.x() || actual.y()!=expected.y())
                        throw std::runtime_error("getPosition arithmetic mismatch");
                    for(double dx:{-0.501,-0.5,-0.499,0.0,0.499,0.5,0.501}) {
                        const double x=actual.x()+dx*resolution, y=actual.y()+dx*resolution;
                        grid_map::Index a; groundgrid::PlanningIndex b;
                        const bool av=map.getIndex(grid_map::Position(x,y),a), bv=plain.getIndex({x,y},b);
                        if(av!=bv || (av && (a(0)!=b.a || a(1)!=b.b)))
                            throw std::runtime_error("getIndex boundary/buffer mismatch");
                    }
                }
            }
        }
        std::cout<<"planning_grid_parity_selfcheck passed\n";
        return 0;
    } catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
