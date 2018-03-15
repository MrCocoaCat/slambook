
// 04/12/2015
// by suiwei
// opeartions about saving to PLY files or Load Points from PLY files
// ply files may be different accroding to the type of the points, e.g., three atrributes, color, normal, or poistions
// and also mesh should be considered.

#ifndef IGIT_DATAIO_H
#define IGIT_DATAIO_H
#include"data_type.h"
#include <QObject>
#include<QString>
#include<QVector>
#include<QVector3D>
#include<QVector2D>

class DATAIO
{
public:
	DATAIO(){}
	DATAIO(QString name):file_dir_(name){}

	QVector<Point> loadPointsFromPLY(QString fileName);

	QVector<QVector<int> > loadVisibilityFromPatchFile(QString fileName);

	bool SaveTexturedMesh2OBJ();

private:
	QString file_dir_;
};


#endif