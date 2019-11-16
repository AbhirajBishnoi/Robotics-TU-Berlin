#include <QApplication>
#include <Inventor/Qt/SoQt.h>

#include "qt_visualization/QtWindow.h"
#include "TutorialPlanSystem.h"

//Initialize the global singleton variable of the main visualization window with null.
QtWindow* QtWindow::singleton = NULL;

int
main(int argc, char** argv)
{
  //  Create the qt application object needed for the visualization.
  QApplication application(argc, argv);
  QObject::connect(&application, SIGNAL(lastWindowClosed()), &application, SLOT(quit()));

  //  The QtWindow class contains the main window needed for the visualization.
  QtWindow* window;

  //  Initialization of the coin 3D libraries which are needed for the visualization of the robot scenes.
  SoQt::init(window);
  SoDB::init();

  //  Create the TutorialPlanSystem class which contains our roblib plan system.
  boost::shared_ptr<TutorialPlanSystem> system(new TutorialPlanSystem());

  //  Create our main visualization window and pass our TutorialPlanSystem to the constructor.
  window = QtWindow::instance(system.get());

  //  Show the main visualization window to the user.
  window->show();

  //  Run the qt application.
  return application.exec();
}
