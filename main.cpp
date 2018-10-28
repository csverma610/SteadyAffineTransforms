#include "AffineMotion.h"
#include <qapplication.h>

int main(int argc, char **argv)
{
    assert( argc == 3);
    QApplication application(argc, argv);

    // Instantiate the viewer.
    AffineMotion viewer;

    viewer.setWindowTitle("MeshCutter");
    viewer.readMesh( argv[1] );
    viewer.readAffinityMatrix( argv[2] );

    // Make the viewer window visible on screen.
    viewer.show();

    // Run main loop.
    return application.exec();
}
