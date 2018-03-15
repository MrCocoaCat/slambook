#ifndef IGIT_MESH_GENERATION_H
#define IGIT_MESH_GENERATION_H

#include"data_type.h"
#include"triangle.h"
#include<pthread.h>

#include <QVector2D>
#include<QVector>

typedef PointXYZRGBNormal Point;

//------------------------------------------------Mesh Generation Based Class----------------------------------------------------------//
class MeshGenerationBase 
{

public:

	MeshGenerationBase(){}
	~MeshGenerationBase(){
		delete p_dense_pts_;
		//delete p_points_ids_;
		delete p_vertices_;
		delete p_facets_;
		//delete p_edges_;
	}
    void setMeshVeticesPtr(QVector<Point>* ptr){ p_vertices_ = ptr; }
    void setMeshFacetPtr(QVector<QVector<int> > * ptr){ p_facets_ = ptr; }
	//void setMeshEdgesPtr(QVector<QPair<int, int> >* ptr){ p_edges_ = ptr; }
    void setDensePointsPtr(QVector<Point>* ptr){
		p_dense_pts_ = ptr;
	}
    /*void setPointsIdsPtr(QSet<int> *ptr){
		p_points_ids_ = ptr;
	}*/

	// trim facets
	virtual bool trimmingFacets(){ return true;}

	// some vertices are eliminated and then the mesh should be updated from the facets infomation
    void updateMesh(QVector<QVector<Point> > & facets);

    bool loadMeshFromPLY(QString file_name);

	// save points to PLY for Poisson Reconstruction
    bool savePointsToPLYFile(QString file_name);

	virtual void run(){}

public:
	// dense points
	QVector<Point>* p_dense_pts_;

	// points indices
	//QSet<int> * p_points_ids_;

	// vertices of the mesh
	QVector<Point> *p_vertices_;

	// facets of the edges
	QVector<QVector<int> > *p_facets_;


	// edges of the mesh
	//QVector<QPair<int, int> > *p_edges_;
};

//------------------------------------------------Poisson Surface Reconstruction-------------------------------------------------------//
class Poisson : public MeshGenerationBase
{
public:
	Poisson(int depth = 8){
		NumOfThreads_=8;
	    JobUint_ = 100;
		depth_=depth;
	}

	void trimmingFacetsThread();
	static void* trimmingFacetsThreadTmp(void *arg);
	bool trimmingFacets();
    void setDepth(int depth){depth_ = depth;}
//public slots:
	void run();
public:
	QVector<double> MaxValues_;
	QVector<double> MinValues_;
	QVector<bool> Trimmered_;
	int NumOfThreads_;
	pthread_rwlock_t Lock_;
	QVector<int> Jobs_;
	int JobUint_;

	// depth for poisson reconstruction
	int depth_;
};


//------------------------------------------------Delaunay Triangulation---------------------------------------------------------------//
class DelaunayTriangulation : public MeshGenerationBase
{
	//Q_OBJECT

public:
	DelaunayTriangulation(){}
	bool trimmingFacets();
	void setFacetsNeighboursPtr(QVector<QVector<int> > * ptr){ p_facets_neighbours_ = ptr; }

//public slots:

    bool delaunay2Mesh(triangulateio *in, string ErrorStr);
	bool run(QString ErrorStr);
	bool collectMesh(triangulateio *in);

private:
	Plane3D p_plane_;
	QVector<QVector2D > p_points2D_;
	QVector<QVector<int> > *p_facets_neighbours_;
};

#endif