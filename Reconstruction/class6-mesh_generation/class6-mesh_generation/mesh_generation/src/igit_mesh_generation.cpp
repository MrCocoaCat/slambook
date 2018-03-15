#include"igit_mesh_generation.h"
#include<qdir.h>
#include<qtextstream.h>
#include "PoissonRecon.h"
#include<ctime>

/////////////////////////////////////loadMeshFromPLY////////////////////////////////////////////////////////////
bool MeshGenerationBase::loadMeshFromPLY(QString file_name){

	//----------------------- read vertices and facets from file ---------------------//
	QFile file(file_name);
	if (!file.open(QIODevice::ReadOnly))	{
		//emit textEdit(tr("Fail to Load Sparse Points!"));
		return false;
	}
	QTextStream in(&file);

	int facet_num = 0;
	int vertex_num = 0;
	int counter = 0;
	bool start_read = false;
	while (!in.atEnd())
	{
		QString line = in.readLine();
		QStringList fields = line.split(" ");

		if (line.startsWith("ply") || line.startsWith("comment") || line.startsWith("format"))	{
			continue;
		}

		// nummber of vetices
		if (line.startsWith("element vertex")){
			fields.takeFirst();
			fields.takeFirst();
			vertex_num = fields.takeFirst().toInt();
			continue;
		}

		// nummber of facets
		if (line.startsWith("element face")){
			fields.takeFirst();
			fields.takeFirst();
			facet_num = fields.takeFirst().toInt();
		}

		// end of header
		if (line.startsWith("end_header")){
			start_read = true;
			counter = 0;
			continue;
		}

		// read vertices
		if (start_read == true && counter < vertex_num){
			Point pt;
			pt.x = fields.takeFirst().toFloat();
			pt.y = fields.takeFirst().toFloat();
			pt.z = fields.takeFirst().toFloat();
			p_vertices_->append(pt);
			counter++;
		}

		// read facets
		if (start_read == true && counter >= vertex_num && counter < vertex_num + facet_num +1){
			if (counter == vertex_num){
				counter++;
				continue;
			}
			QVector<int> facet;
			int num = fields.takeFirst().toInt();

			for (int i = 0; i < num; i++){
				facet.append(fields.takeFirst().toInt());
			}
			p_facets_->append(facet);
			counter++;
		}
	}
	//--------------------------------compute edges -----------------------------------------//
	// get edges from facets
	/*QSet<QPair<int, int> > e;
	foreach(QVector<int> facet, *p_facets_)
	{
		int pt_num = facet.size();
		for (int i = 0; i < pt_num; i++)
		{
			int id0 = facet[i];
			int id1 = facet[(i + 1) % pt_num];

			if (id0 > id1)
			{
				e << qMakePair(id1, id0);
			}
			else{
				e << qMakePair(id0, id1);
			}
		}
	}
	QSet<QPair<int, int> > ::const_iterator iter = e.constBegin();
	while (iter != e.constEnd())
	{
		(*p_edges_) << *iter;
		iter++;
	}*/

	return true;
}


////////////////////////////////////////savePointsToPLYFile////////////////////////////////////////////////
bool MeshGenerationBase::savePointsToPLYFile(QString file_name)
{
	QFile file(file_name);
	if (!file.open(QIODevice::WriteOnly))
	{
		//statusBar(tr("Fail to Save PLY!"));
		return false;
	}

	QTextStream out(&file);
	out << "ply" << endl;
	out << "format ascii 1.0" << endl;
	out << "element vertex " << p_dense_pts_->size() << endl;
	out << "property float x" << endl;
	out << "property float y" << endl;
	out << "property float z" << endl;
	out << "property float nx" << endl;
	out << "property float ny" << endl;
	out << "property float nz" << endl;
	out << "end_header" << endl;

	foreach(Point pt, *p_dense_pts_){
	 out << pt.x << " " << pt.y << " " << pt.z << " ";
	 out << pt.normal_x << " " << pt.normal_y << " " << pt.normal_z << endl;
	}
	return true;
}


/////////////////////////////////////////updateMesh///////////////////////////////////////////////////////////
void MeshGenerationBase::updateMesh(QVector<QVector<Point> > & facets)
{
	// create  a table  for inquerying the new index of vertex
	// Note: we can do this because the structure of Point is special defined(overlod of operator < in "data_type.h").
	// and may not work for other type of structure.
	map<Point, int> table;
	foreach(QVector<Point> facet, facets){
		foreach(Point pt, facet){
			table.insert(make_pair(pt, 0));
		}
	}

	//attach index to each point
	int index = 0;
	for (map<Point, int> ::iterator iter = table.begin(); 
		                            iter != table.end(); iter++){
		iter->second = index;
		index++;
	}

	QVector<Point> new_vertices;
	QVector<QVector<int> > new_facets;
	QVector<QPair<int, int> > new_edges;

	// get new vertices
	for (map<Point, int> ::iterator iter = table.begin(); iter != table.end(); iter++){
		new_vertices.append(iter->first);
	}

	// get new facets
	foreach(QVector<Point> facet, facets){
		QVector<int> facetID;
		foreach(Point pt, facet){
			facetID.append(table[pt]);
		}
		int l0 = facetID[0];
		int l1 = facetID[1];
		int l2 = facetID[2];
		if(l0==l1||l1==l2||l0==l2) continue;
		new_facets.append(facetID);
	}

	// get new edges
	/*QSet<QPair<int, int> > e;
	foreach(QVector<int> facet, new_facets)
	{
		int pt_num = facet.size();
		for (int i = 0; i< pt_num; i++)
		{
			int id0 = facet[i];
			int id1 = facet[(i + 1) % pt_num];

			if (id0< id1) e.insert(qMakePair(id0, id1));
			if (id1< id0) e.insert(qMakePair(id1, id0));
		}
	}
	QSet<QPair<int, int> > ::const_iterator iter = e.constBegin();
	while (iter != e.constEnd())
	{
		new_edges << (*iter);
		iter++;
	}
*/

	p_vertices_->swap(new_vertices);
	//p_edges_->swap(new_edges);
	p_facets_->swap(new_facets);
}
////////////////////////////////////////trimmingFacetsThread///////////////////////////////////////////////////
void Poisson::trimmingFacetsThread(){

	int size = p_vertices_->size();
	while (1) {
		int jtmp = -1;
		pthread_rwlock_wrlock(&Lock_);
		if (!Jobs_.empty()){
			jtmp = Jobs_.front();
			Jobs_.pop_front();
		}
		pthread_rwlock_unlock(&Lock_);
		if (jtmp == -1) break;

		const int begin = JobUint_ * jtmp;
		const int end = min(size, JobUint_*(jtmp + 1));

		for(int i=begin; i< end; i++){
			pthread_rwlock_wrlock(&Lock_);
		    QVector<int> facet = (*p_facets_)[i];
		    pthread_rwlock_unlock(&Lock_);

			int counter = 0;
			foreach(int vid, facet){
			    pthread_rwlock_wrlock(&Lock_);
		        Point pt = (*p_vertices_)[vid];
		        pthread_rwlock_unlock(&Lock_);
				if (pt.x<MinValues_[0] || pt.x> MaxValues_[0] || pt.y < MinValues_[1] || pt.y>MaxValues_[1] || pt.z < MinValues_[2] || pt.z >MaxValues_[2]){
					counter++;
				}
			}
			if(counter <3) continue;
			pthread_rwlock_wrlock(&Lock_);
		    Trimmered_[i] = true;
		    pthread_rwlock_unlock(&Lock_);
		}
	}
}
////////////////////////////////////////trimmingFacetsThreadTmp///////////////////////////////////////////////////
void* Poisson::trimmingFacetsThreadTmp(void * arg){
	((Poisson*)arg)->trimmingFacetsThread();
	 return NULL;
}
////////////////////////////////////////trimmer/////////////////////////////////////////////////////////////
bool Poisson::trimmingFacets(){
	
	//-----------------------------elimiate vertices exceeding the bounding box--------------------------------//
	MaxValues_<<-1e5<<-1e5<<-1e5;
	MinValues_<<1e5<<1e5<<1e5;

	// bounding box are computed from the dense points
	foreach(Point pt, *p_dense_pts_){
		if (MinValues_[0] > pt.x) MinValues_[0] = pt.x;
		if (MinValues_[1] > pt.y) MinValues_[1] = pt.y;
		if (MinValues_[2] > pt.z) MinValues_[2] = pt.z;

		if (MaxValues_[0] < pt.x) MaxValues_[0] = pt.x;
		if (MaxValues_[1] < pt.y) MaxValues_[1] = pt.y;
		if (MaxValues_[2] < pt.z) MaxValues_[2] = pt.z;
	}
	for(int i=0; i<p_facets_->size(); i++) Trimmered_.append(false);

	Jobs_.clear();
	const int jtmp = (int)ceil((double)p_facets_->size() / (double)JobUint_);
	for (int j = 0; j< jtmp; j++) Jobs_.append(j);

	// WR Lock must be initialized!
	if (pthread_rwlock_init(&Lock_, NULL)){
		return false;
	}
	pthread_t * threads = new pthread_t[NumOfThreads_];
	for (int i = 0; i< NumOfThreads_; i++){
		pthread_create(&threads[i], NULL, trimmingFacetsThreadTmp, (void*)this);
	}

	for (int i = 0; i< NumOfThreads_; i++){
		pthread_join(threads[i], NULL);
	}

	QVector<QVector<Point> > all_facets;
	for(int i=0; i< Trimmered_.size(); i++){
		if(Trimmered_[i] == true) continue;
		QVector<int> fIds = (*p_facets_)[i];
		QVector<Point> points;
		foreach(int vid, fIds){
			points.append((*p_vertices_)[vid]);
		}
		all_facets.append(points);
	}
	//// trimmed facets
//	QVector<int> trimmed_vertices;
//	int counter = 0;
//	foreach(Point pt, *p_vertices_)	{
//		if (pt.x<min_x || pt.x> max_x || pt.y < min_y || pt.y>max_y || pt.z < min_z || pt.z >max_z)	{
//			trimmed_vertices.append(counter);
//		}
//		counter++;
//	}

	//// check whether the facets contain eliminated vertice, and if true the facets are eliminated
	//QVector<QVector<Point> > all_facets;
	//foreach(QVector<int> facet, *p_facets_)	{
	//	QVector<Point> facets;
	//	bool trimmed = false;
	//	foreach(int id, facet){
	//		facets.append((*p_vertices_)[id]);
	//		if (trimmed_vertices.contains(id)){
	//			trimmed = true;
	//			break;
	//		}
	//	}

	//	if (trimmed == false){
	//		all_facets.append(facets);
	//	}
	//}

	//--------------------------------------------------update the mesh---------------------------------------//
	// reconstruct the mesh from the remaining facets
	updateMesh(all_facets);
	return true;
}

/////////////////////////////////////run ////////////////////////////////////////////////////////////////////////
void Poisson::run()
{
	const double begin = (double)clock() / CLOCKS_PER_SEC;
	//emit textEdit("[Run Possion Surface Reconstruction\n");
	//emit statusBar("Run Possion Surface Reconstruction");

	
	savePointsToPLYFile("tmp_points.ply");

	//QString cmdAgu("PoissonRecon.exe --in all_points.ply  --out mesh.ply");
	//emit textEdit(cmdAgu);
	//system("PoissonRecon.exe --in all_points.ply --depth 10 --out mesh.ply --ascii");
    poisson("tmp_points.ply", "mesh.ply", depth_);
#if 0
	QProcess cmd;
	cmd.start("Bundler.exe list.txt --options_file options.txt > bundler.out");
	while (cmd.waitForFinished())
	{
	}
# endif
	//emit textEdit("  [Loading Meshes...");
	loadMeshFromPLY("mesh.ply");
	//emit textEdit("  Done!] \n");


	//---------------------------------------trimming facets--------------------------------------------------//
	// therare many facets that are generated by interplotation and there are no points support them , and we trim
	// these facets by first computing a bounding box of the dense points and any vertices exceed this bounding box
	// are deleted. It is a very easy trick, and can not work for all the situations

	//emit textEdit("  [Trimming Facets...");
	//trimmingFacets();
	//emit textEdit("  Done!]\n");

	QFile file;
	if(file.exists("tmp_points.ply"))file.remove("tmp_points.ply");
	if(file.exists("mesh.ply"))file.remove("mesh.ply");


	const double end = (double)clock() / CLOCKS_PER_SEC;
	QString txt = QString("Running time is %1 s").arg(end - begin);
//	textEdit(txt);

}


/////////////////////////////////////////trimmer facets//////////////////////////////////////////////////////////
bool DelaunayTriangulation::trimmingFacets()
{
	return true;
}

/////////////////////////////////////////delaynay2Mesh////////////////////////////////////////////////////////////
bool DelaunayTriangulation::delaunay2Mesh(triangulateio *in, string ErrorStr)
{
	uint PtNum = in->numberofpoints;

	if (PtNum < 3){
		ErrorStr = "Not Enough Points";
		return false;
	}

	try{
		triangulate("zQNn", in, in, 0);
	}
	catch (std::exception & e){
		ErrorStr = e.what();
		return false;
	}
	catch (...){
		ErrorStr = "Unknown error";
		return false;
	}
	return true;

}
/////////////////////////////////////collectMesh//////////////////////////////////////////////////////////////////
bool  DelaunayTriangulation::collectMesh(triangulateio *in)
{
	// vertices
	p_vertices_->resize(p_dense_pts_->size());
	int count = 0;
	for (int id = 0; id < p_dense_pts_->size(); id++){
		(*p_vertices_)[id] = (*p_dense_pts_)[id];
	}

	// collect facets
	for (int i = 0; i < in->numberoftriangles; i++){
		QVector<int> facet;
		facet << in->trianglelist[i * 3 + 0];
		facet << in->trianglelist[i * 3 + 1];
		facet << in->trianglelist[i * 3 + 2];
		p_facets_->append(facet);
	}

	// collect facet neigbours for MRF visibility opimiztion
//	p_facets_neighbours_->clear();
//	p_facets_neighbours_->reserve(in->numberoftriangles);
//
//	for (int i = 0; i < in->numberoftriangles; i++){
//		QVector<int> neigh;
//		neigh << in->neighborlist[i * 3 + 0];
//		neigh << in->neighborlist[i * 3 + 1];
//		neigh << in->neighborlist[i * 3 + 2];
//		p_facets_neighbours_->append(neigh);
//	}

	//--------------------------------compute edges -----------------------------------------//
	// get edges from facets
	/*QSet<QPair<int, int> > e;
	foreach(QVector<int> facet, *p_facets_){
		int pt_num = facet.size();
		for (int i = 0; i < pt_num; i++){
			int id0 = facet[i];
			int id1 = facet[(i + 1) % pt_num];

			if (id0 > id1){
				e << qMakePair(id1, id0);
			}
			else{
				e << qMakePair(id0, id1);
			}
		}
	}
	QSet<QPair<int, int> > ::const_iterator iter = e.constBegin();
	while (iter != e.constEnd()){
		(*p_edges_) << *iter;
		iter++;
	}*/
	return true;
}

////////////////////////////////////////run()/////////////////////////////////////////////////////////////////////
bool  DelaunayTriangulation::run(QString ErrorStr)
{
	const double begin = (double)clock() / CLOCKS_PER_SEC;
	//emit textEdit("[Run Delaunay Triangulation \n");
	//emit statusBar("Run Delaunay Triangulation");

	p_plane_.fittingPlane(*p_dense_pts_);

	// project all the vertices onto the plane
	p_points2D_.reserve(p_dense_pts_->size());
	foreach(Point pt3D, *p_dense_pts_){
		QPointF pt = p_plane_.cvt3Dto2D(pt3D);
		p_points2D_ << QVector2D(pt.x(), pt.y());
	}
	/*foreach(int id, *p_points_ids_)
	{
		QPointF pt = p_plane_.cvt3Dto2D((*p_dense_pts_)[id]);
		p_points2D_ << QVector2D(pt.x(), pt.y());
	}*/

	triangulateio  * in = new triangulateio();
	in->numberofpoints = static_cast<int>(p_points2D_.size());
	in->pointlist = (TREAL*)(p_points2D_.constData());

	string str;
	if (!delaunay2Mesh(in, str)){
		ErrorStr = QString(str.c_str());
		return false;
	}

	// collect information
	p_vertices_->clear();
	p_facets_->clear();
//	p_edges_->clear();

	collectMesh(in);

	delete in;
	//emit textEdit("Done!]");
	//emit statusBar("Done!");
	//emit enableActionMesh();
	const double end = (double)clock() / CLOCKS_PER_SEC;
	QString txt = QString("Running time is %1 s").arg(end - begin);
	//textEdit(txt);

	return true;
}