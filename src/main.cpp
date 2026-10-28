//
//  main.cpp
//  Loop
//
//  Created by S WEI on 16/7/20.
//  Copyright © 2016年 S WEI. All rights reserved.
//
// #include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>


#include "Window.hpp"
#include "Loop.hpp"
#include "Sqrt_3.hpp"
#include <unistd.h>
#include <limits>

namespace PMP = CGAL::Polygon_mesh_processing;

void usage()
{
    std::cerr<<"Accept option and value: \n"
    <<"[-a ]:Enable adaptive mode\n"
    <<"[-m $(Method)]:Accetp either loop or sqrt3\n"
    <<"[-l $(max_levels)]:Terminate condition in terms of maximum levels,by default, it is max int\n"
    <<"[-f $(max_faces)]:Terminate condition in terms of maximum faces, by default, it is max int\n"
    <<"[-t $(angle)]:Dihedral tolerance\n";
    std::exit(1);
}


double convertDouble(const std::string& arg)
{
    std::size_t n(0);
    
    double v;
    try{
        v = std::stod(arg, &n);
    }catch(...){
        std::cerr<<arg<<" is not a valid value\n";
        std::exit(1);
    }
    
    if(n!=arg.size())
    {
        std::cerr<<arg<<" is not a valid value\n";
        std::exit(1);
    }
    
    return v;
}

int convertInt(const std::string& arg)
{
    std::size_t n(0);
    
    double v;
    try{
        v = std::stoi(arg, &n);
    }catch(...){
        std::cerr<<arg<<" is not a valid value\n";
        std::exit(1);
    }
    
    if(n!=arg.size())
    {
        std::cerr<<arg<<" is not a valid value\n";
        std::exit(1);
    }
    
    return v;
}


//Process function for computation and rendering 
template<class T>
void process(T& mesh, const double& angle, bool adaptive, const int& maxLevel, const int& maxFaces, std::string inputFilename, std::string outputFilename)
{
    
    std::cout<<"Precomputing...\n";
    mesh.loadMesh(inputFilename);
    
    mesh.setCriteria(angle);
    if(adaptive)
        mesh.enableAdaptive();
    

    
    int Level = 0;

    
    std::cout << "Start Loop" << std::endl;
    while(mesh.size_of_faces()<maxFaces && Level < maxLevel)
    {
        std::cout << "Curent Level : " << Level << std::endl;
        std::size_t before=mesh.size_of_faces();

        mesh.execution();
        ++Level;
    

        
        double rise =static_cast<double>(mesh.size_of_faces()-before)/static_cast<double>(mesh.size_of_faces());
       
        if(rise<0.0001){
            break;
        }
    }
    std::cout<<"Maximum faces: "<<mesh.size_of_faces()<<"\n";
    mesh.writeMesh(outputFilename);

    
}


int main(int argc,char** argv)
{
    char c;
    
    //Default subdivision mode
    bool enableAdaptive = false;
    
    //Default dihedral tolerance
    double angle=0.0;
    
    //Default subdivision level limitation
    int maxlevel=std::numeric_limits<int>::max();
    
    //Default number of faces limitation
    int maxfaces=std::numeric_limits<int>::max();
    std::string inputFilename ;
    std::string outputFilename ;
    
    std::string method;
    
    while((c=getopt(argc,argv,"at:l:m:f:o:i:"))!=EOF)
    {
        switch(c){
                case 'o':
                outputFilename = std::string(optarg) ;
                break;

                case 'i':
                inputFilename = std::string(optarg) ;
                break;

                case 'a':
                enableAdaptive=true;
                break;
                
                case 't':
                angle = convertDouble(optarg);
                break;
                
                case 'l':
                maxlevel = convertInt(optarg);
                break;
                
                case 'f':
                maxfaces = convertInt(optarg);
                break;
                
                case 'm':
                method = std::string(optarg);
                if(method != "loop" && method != "sqrt3"){
                    usage();
                }
                break;
                
            default:
                usage();
                break;
                
                
        }
    }
        
    
    if(method == ""){
        usage();
    }
   


    
    if(method == "loop")
    {
        Loop loop;
        process(loop,angle,enableAdaptive,maxlevel,maxfaces,inputFilename,outputFilename);
    }else if(method == "sqrt3")
    {
        Sqrt_3 sqrt;
        process(sqrt,angle,enableAdaptive,maxlevel,maxfaces,inputFilename,outputFilename);
    }
    
    
    return 0;
}
