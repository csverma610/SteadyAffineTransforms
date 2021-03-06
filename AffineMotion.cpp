#include "AffineMotion.h"

using namespace std;

////////////////////////////////////////////////////////////////////////////////
void AffineMotion::init()
{
    setSelectRegionWidth(10);
    setSelectRegionHeight(10);
}
////////////////////////////////////////////////////////////////////////////////

EdgePtr Mesh:: addEdge( NodePtr &n0, NodePtr &n1, FacePtr &face)
{
    NodePtr vmin = std::min(n0,n1);

    for( auto oldedge : vmin->edges) {
        if( oldedge->hasNodes(n0,n1)) {
            oldedge->faces[1] = face;
            return oldedge;
        }
    }

    EdgePtr newedge(new Edge(n0,n1));
    newedge->faces[0] = face;
    vmin->edges.push_back(newedge);
    edges.push_back(newedge);
    return newedge;
}
////////////////////////////////////////////////////////////////////////////////

void Mesh::addFace( FacePtr &newface)
{
    assert( newface );
    auto n0 = newface->nodes[0]; assert( n0 );
    auto n1 = newface->nodes[1]; assert( n1 );
    auto n2 = newface->nodes[2]; assert( n2 );
    assert((n0 != n1) && (n1 != n2) && (n2 != n0));

    newface->edges[0] = addEdge(n0,n1,newface);
    newface->edges[1] = addEdge(n1,n2,newface);
    newface->edges[2] = addEdge(n2,n0,newface);

    n0->faces.push_back(newface);
    n1->faces.push_back(newface);
    n2->faces.push_back(newface);

    faces.push_back(newface);
}

////////////////////////////////////////////////////////////////////////////////
void Mesh::setSurfaceNormals()
{
    for( auto f: faces) {
        auto p0 = f->nodes[0]->xyz;
        auto p1 = f->nodes[1]->xyz;
        auto p2 = f->nodes[2]->xyz;
        f->normal = normal(p0,p1,p2);
    }
}

void AffineMotion:: readMesh( const string &filename)
{
    ifstream ifile( filename.c_str(), ios::in);
    if( ifile.fail() ) {
        cout << "Warning: Input file not read " << endl;
        return;
    }
    string str;
    ifile >> str;
    if( str != "OFF") {
        cout << "Warning: Input file not in Off format" << endl;
        return;
    }

    size_t numNodes, numFaces, numEdges;
    ifile >> numNodes >> numFaces >> numEdges;

    double x, y, z;

    srcmesh.nodes.resize(numNodes);
    for( size_t i = 0; i < numNodes; i++) {
        ifile >> x >> y >> z;
        NodePtr v = Node::newObject();
        v->xyz[0] = x;
        v->xyz[1] = y;
        v->xyz[2] = z;
        v->id     = i;
        srcmesh.nodes[i] = v;
    }

    size_t index = 0;
    int dummy, v0, v1, v2;

    for( size_t i = 0; i < numFaces; i++) {
        ifile >> dummy >> v0 >> v1 >> v2;
        assert( dummy == 3);
        NodePtr n0   = srcmesh.nodes[v0];
        NodePtr n1   = srcmesh.nodes[v1];
        NodePtr n2   = srcmesh.nodes[v2];
        FacePtr newface = Face::newObject(n0,n1,n2);
        newface->id     = i;
        srcmesh.addFace(newface);
    }

    currmesh.nodes.resize(numNodes);
    dstmesh.nodes.resize(numNodes);

    NodePtr v;
    for( int i = 0; i < numNodes; i++) {
        v = Node::newObject();
        v->xyz    = srcmesh.nodes[i]->xyz;
        v->id     = i;
        currmesh.nodes[i] = v;

        v = Node::newObject();
        v->xyz    = srcmesh.nodes[i]->xyz;
        v->id     = i;
        dstmesh.nodes[i] = v;
    }

    currmesh.faces.resize(numFaces);
    dstmesh.faces.resize(numFaces);

    FacePtr newface;
    int n0, n1, n2;
    for( size_t i = 0; i < numFaces; i++) {
        n0 = srcmesh.faces[i]->nodes[0]->id;
        n1 = srcmesh.faces[i]->nodes[1]->id;
        n2 = srcmesh.faces[i]->nodes[2]->id;
        newface = std::make_shared<Face>();
        newface->nodes[0] = currmesh.nodes[n0];
        newface->nodes[1] = currmesh.nodes[n1];
        newface->nodes[2] = currmesh.nodes[n2];
        currmesh.faces[i] = newface;

        newface = std::make_shared<Face>();
        newface->nodes[0] = dstmesh.nodes[n0];
        newface->nodes[1] = dstmesh.nodes[n1];
        newface->nodes[2] = dstmesh.nodes[n2];
        dstmesh.faces[i] = newface;
    }
    dt = 1.0/(double)maxSteps;

    srcmesh.setSurfaceNormals();
    currmesh.setSurfaceNormals();
    dstmesh.setSurfaceNormals();
}

////////////////////////////////////////////////////////////////////////////////

void AffineMotion:: readAffinityMatrix( const string &filename)
{
    ifstream ifile( filename.c_str(), ios::in);

    Eigen::Matrix4d aa;
    for( int i = 0; i < 4; i++)
        ifile >> aa(i,0) >> aa(i,1) >> aa(i,2) >> aa(i,3);

    A = aa.transpose();   // Prof. Shizuo Kaji, the author of AfflineLib helped..

    logA =  AffineLib::logSEc(A);

    startPos = {0.0, 0.0, 0.0};
    endPos   = {A(3,0), A(3,1), A(3,2)};
}

////////////////////////////////////////////////////////////////////////////////

void Mesh::saveAs( const std::string &filename)
{
    ofstream ofile(filename.c_str(), ios::out);

    size_t  numnodes = 0;
    for( auto v: nodes) {
        if( v->active ) v->id = numnodes++;
    }

    set<NodePtr> vSet;
    size_t  numfaces = 0;
    for( auto f: faces) {
        if( f->active ) {
            vSet.insert(f->nodes[0]);
            vSet.insert(f->nodes[1]);
            vSet.insert(f->nodes[2]);
            numfaces++;
        }
    }

    size_t index = 0;
    for( auto v: vSet) {
        v->id = index++;
        ofile << "v " << v->xyz[0] << " " << v->xyz[1] << " " << v->xyz[2] << endl;
    }

    for( auto f: faces) {
        if( f->active )
            ofile << "f " << f->nodes[0]->id +1 << " "
                  << f->nodes[1]->id +1 << " "
                  << f->nodes[2]->id +1 << endl;
    }
}

////////////////////////////////////////////////////////////////////////////////

void AffineMotion::mult( Eigen::Matrix4d &At, Mesh &msh)
{
    int numnodes = srcmesh.nodes.size();
    Eigen::Vector4d vec;

    for( int i = 0; i < numnodes; i++) {
        vec[0] = srcmesh.nodes[i]->xyz[0];
        vec[1] = srcmesh.nodes[i]->xyz[1];
        vec[2] = srcmesh.nodes[i]->xyz[2];
        vec[3] = 1.0;
        auto x = At.transpose()*vec;  // Prof. Kaji helped me here.
        msh.nodes[i]->xyz[0] = x[0];
        msh.nodes[i]->xyz[1] = x[1];
        msh.nodes[i]->xyz[2] = x[2];
    }
}

////////////////////////////////////////////////////////////////////////////////

void AffineMotion::keyPressEvent( QKeyEvent *e)
{
    if( e->key() == Qt::Key_0) {
        pickEntity = 0;
        this->setSelectedName(-1);
    }

    if( e->key() == Qt::Key_N) {
        displayIDs = !displayIDs;
    }

    if( e->key() == Qt::Key_S) {
        displaySurface = !displaySurface;
        update();
        return;
    }

    if( e->key() == Qt::Key_W) {
        displayWires = !displayWires;
    }

    if( e->key() == Qt::Key_L) {
        useLights = !useLights;
        update();
        return;
    }

    if( e->key() == Qt::Key_N) {
        nstep++;
        double t = nstep*dt;
        if( t <= 1.0) {
            At  = AffineLib::expSE(t*logA);
            mult( At, currmesh);
            update();
        }
        return;
    }

    if( e->key() == Qt::Key_R) {
        nstep = 1;
        double t = nstep*dt;
        if( t <= 1.0) {
            At  = AffineLib::expSE(t*logA);
            mult( At, currmesh);
            update();
        }
        return;
    }
    if( e->key() == Qt::Key_Home) {
        qglviewer::Vec pos;
        pos[0]  = srcmesh.center[0];
        pos[1]  = srcmesh.center[1];
        pos[2]  = srcmesh.center[2];
        camera()->setSceneCenter(pos);
        camera()->setSceneRadius(srcmesh.radius);
        camera()->centerScene();
        camera()->showEntireScene();
        update();
        return;
    }

    QGLViewer::keyPressEvent(e);

    update();
}

////////////////////////////////////////////////////////////////////////////////

void AffineMotion:: mousePressEvent( QMouseEvent *e)
{
    QGLViewer::mousePressEvent(e);
}

////////////////////////////////////////////////////////////////////////////////

void AffineMotion::mouseReleaseEvent( QMouseEvent *e)
{
    int id = this->selectedName();

    QGLViewer::mouseReleaseEvent(e);

    update();
}

////////////////////////////////////////////////////////////////////////////////

void AffineMotion::drawFaces(Mesh &themesh)
{
    if( useLights ) glEnable(GL_LIGHTING);

    for ( auto f : themesh.faces) {
        if(f->active) {
            glBegin(GL_TRIANGLES);
	    glNormal3fv( &f->normal[0]);
            glVertex3fv( &f->nodes[0]->xyz[0] );
            glVertex3fv( &f->nodes[1]->xyz[0] );
            glVertex3fv( &f->nodes[2]->xyz[0] );
            glEnd();
        }
    }
}

////////////////////////////////////////////////////////////////////////////////

void AffineMotion::draw()
{
    if( nstep == 0) {
        mult(A, dstmesh);
    }

    glPolygonOffset(1.0,1.0);
    glEnable(GL_POLYGON_OFFSET_LINE);

    glPolygonMode( GL_FRONT_AND_BACK, GL_FILL);
    glColor3f( 1.0, 0.0, 0.0);
    drawFaces(srcmesh);

    glColor3f( 0.0, 0.0, 1.0);
    drawFaces(dstmesh);

    if( nstep ) {
        glColor3f( 0.0, 1.0, 0.0);
        drawFaces(currmesh);
    }
}

////////////////////////////////////////////////////////////////////////////////
