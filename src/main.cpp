#include <optional>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <io2d.h>
#include "route_model.h"
#include "render.h"
#include "route_planner.h"

using namespace std::experimental;

static std::optional<std::vector<std::byte>> ReadFile(const std::string &path)
{   
    std::ifstream is{path, std::ios::binary | std::ios::ate};
    if( !is )
        return std::nullopt;
    
    auto size = is.tellg();
    std::vector<std::byte> contents(size);    
    
    is.seekg(0);
    is.read((char*)contents.data(), size);

    if( contents.empty() )
        return std::nullopt;
    return std::move(contents);
}

int main(int argc, const char **argv)
{    
    // default values
    std::string osm_data_file = "../map.osm";
    float start_x = 10, start_y = 10, end_x = 90, end_y = 90;

    // slim cli - does not validate...
    if( argc > 1 ) {
        for( int i = 1; i < argc; ++i ){
            std::string_view view = argv[i];
            if (++i < argc) {
                if (view == "map" || view =="-f")
                    osm_data_file = argv[i];
                // seemed easiest to run as cli
                // else if (view == "-x" || view == "-x_start")
                //     start_x = std::stof(argv[i]);
                // else if (view == "-y" || view == "-y_start")
                //     start_y = std::stof(argv[i]);
                // else if (view == "-xg" || "-x_goal")
                //     end_x = std::stof(argv[i]);
                // else if (view == "-yg" || "-y_goal")
                //     end_y = std::stof(argv[i]);
            }
        }
    }
    else {
        std::cout << "To specify a map file use the following format: " << std::endl;
        std::cout << "Usage: [executable] [-f filename.osm]" << std::endl;
        osm_data_file = "../map.osm";
    }
    
    std::vector<std::byte> osm_data;
 
    if( osm_data.empty() && !osm_data_file.empty() ) {
        std::cout << "Reading OpenStreetMap data from the following file: " <<  osm_data_file << std::endl;
        auto data = ReadFile(osm_data_file);
        if( !data )
            std::cout << "Failed to read." << std::endl;
        else
            osm_data = std::move(*data);
    }
    
    // TODO 1: Declare floats `start_x`, `start_y`, `end_x`, and `end_y` and get
    // user input for these values using std::cin. Pass the user input to the
    // RoutePlanner object below in place of 10, 10, 90, 90.

    // Build Model.
    RouteModel model{osm_data};
    std::cout << "input start_x" << std::endl;
    std::cin >> start_x;
    std::cout << "input start_y" << std::endl;
    std::cin >> start_y;
    std::cout << "input end_x" << std::endl;
    std::cin >> end_x;
    std::cout << "input end_x" << std::endl;
    std::cin >> end_x;

    std::cout << "start_x: " << start_x << " start_y: "  << start_y << " end_x: " << end_x << " end_y: " << end_y << std::endl;

    // Create RoutePlanner object and perform A* search.
    RoutePlanner route_planner{model, start_x, start_y, end_x, end_y};
    route_planner.AStarSearch();

    std::cout << "Distance: " << route_planner.GetDistance() << " meters. \n";

    // Render results of search.
    Render render{model};

    auto display = io2d::output_surface{400, 400, io2d::format::argb32, io2d::scaling::none, io2d::refresh_style::fixed, 30};
    display.size_change_callback([](io2d::output_surface& surface){
        surface.dimensions(surface.display_dimensions());
    });
    display.draw_callback([&](io2d::output_surface& surface){
        render.Display(surface);
    });
    display.begin_show();
}
