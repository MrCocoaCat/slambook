#include"igit_dataIO.h"
#include<qfile.h>
#include<qtextstream.h>

bool DATAIO::SaveTexturedMesh2OBJ(){

	return true;
}

//===============================load Points ===================================//
QVector<Point> DATAIO::loadPointsFromPLY(QString fileName){
	
	QVector<Point> points;
	QFile file(fileName);
	if (!file.open(QIODevice::ReadOnly)){
		return points;
	}
	QTextStream in(&file);
	int line_Num = 0;
	while (!in.atEnd()){

		QString line = in.readLine();
		if(line.startsWith("ply"))continue;
		if(line.startsWith("format"))continue;
		if(line.startsWith("element"))continue;
		if(line.startsWith("property"))continue;
		if(line.startsWith("end_header"))continue;
	    QStringList fields = line.split(" ");

	   // points begin
	//	if (line_Num > 12){
		    Point pt;

		    pt.x = fields.takeFirst().toFloat();
		    pt.y = fields.takeFirst().toFloat();
		    pt.z = fields.takeFirst().toFloat();

		    pt.normal_x = fields.takeFirst().toFloat();
			pt.normal_y = fields.takeFirst().toFloat();
			pt.normal_z = fields.takeFirst().toFloat();

			//pt.r = fields.takeFirst().toInt();
			//pt.g = fields.takeFirst().toInt();
			//pt.b = fields.takeFirst().toInt();

			points.append(pt);
			//}
			//line_Num++;
		}
	return points;
}

//===============================load visibility===============================//
QVector<QVector<int> >  DATAIO::loadVisibilityFromPatchFile(QString fileName){
	QVector<QVector<int> > vis;
	QFile file(fileName);
	if (!file.open(QIODevice::ReadOnly | QIODevice::Text)){	
#if DEBUG_
		cout<<"Faile to load file "<< fileName.toStdString()<<endl;
#endif
		return vis;
	}
	else{
		QTextStream in(&file);
		int line_num = 0;
		int counter = 0;
		while (!in.atEnd()){
			QString line = in.readLine();
			QStringList fields = line.split(" ");
			if (line_num == 1){
				int points_num = fields.takeFirst().toInt();
			}
			if (line_num > 1){
				if (line.startsWith("PATCH")){
					counter = 0;
				}
				if (counter == 5){
					QVector<int> tmp;
					while (fields.size() != 0){
						tmp.append(fields.takeFirst().toInt());
					}
					tmp.pop_back();
					vis.append(tmp);
				}
				counter++;
			}
			line_num++;
		}
		return vis;
	}
}