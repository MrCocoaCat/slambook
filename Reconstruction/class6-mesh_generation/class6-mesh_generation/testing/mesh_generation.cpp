//
// Created by swayfreeda on 17-11-30.
//

/* 集成两种网格重建的方法，泊松表面重建算法和德劳内重建方法
 * 泊松重建方法参见网站 http://hhoppe.com/proj/poissonrecon/
 * 泊松表面重建算法重建全局隐函数，因此可以对任意形状的物体进行网格重建
 * 德劳内三角剖分首先将点云投影到二维平面，进行剖分后返回到三维空间，因此对物体形状有一定的要求
 */
#include "igit_mesh_generation.h"
#include "igit_dataIO.h"
#include "data_type.h"

int main(int argc, char* argv[])
{

    if(argc<2) {
        std::cout<<"usage: testing_mesh_generation input_points.ply"<<std::endl;
        return -1;
    }



    /****************1.0 load points ***************/
   //[NOTICE]: for poisson surface reconstrucion, points must contain normals as input attribute
    DATAIO dataIn;
    QVector<Point> all_points = dataIn.loadPointsFromPLY(argv[1]);
    std::cout<<all_points.size()<<" points loaded in total"<<std::endl;


    /****************2.0 poissson surface reconstruction ***************/
    std::cout<<"start poisson surface reconstruction..."<<std::endl;
    clock_t  time_begin = clock();
    Mesh poisson_mesh;
    Poisson * PoissonRec = new Poisson();
    PoissonRec->setDensePointsPtr(&all_points);
    PoissonRec->setMeshVeticesPtr(&poisson_mesh.Vertices_);
    PoissonRec->setMeshFacetPtr(&poisson_mesh.Facets_);
    PoissonRec->run();
    clock_t  time_end = clock();
    std::cout<<" poisson surface reconstruction cost "<< double(time_end-time_begin)/(double)CLOCKS_PER_SEC<<" s"<<std::endl;
    // save mesh to file
    poisson_mesh.saveMeshAsOFF("poisson_mesh.off");


    /****************3.0 load delaunay triangulation ***************/
    std::cout<<"start delaunay triangulation..."<<std::endl;
    time_begin = clock();
    Mesh dt_mesh;
    DelaunayTriangulation *DT = new DelaunayTriangulation();
    QSet<int> PtIds;
    for (int i = 0; i < all_points.size(); i++){
        PtIds.insert(i);
    }
    DT->setDensePointsPtr(&all_points);
    DT->setMeshVeticesPtr(&dt_mesh.Vertices_);
    DT->setMeshFacetPtr(&dt_mesh.Facets_);
    QString Error;
    if (!DT->run(Error)){
        std::cout<<"Delaunay Triangulation Error: "<<Error.toStdString()<<std::endl;
        return -1;
    }
    time_end = clock();
    std::cout<<" delaunay triangulaton cost "<< double(time_end-time_begin)/(double)CLOCKS_PER_SEC<<" s"<<std::endl;
    // save mesh to file
    dt_mesh.saveMeshAsOFF("dt_mesh.off");


    return 0;
}