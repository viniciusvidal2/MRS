/**
 * @file /include/guiROS/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef guiROS_QNODE_HPP_
#define guiROS_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/videoio.hpp"


/*****************************************************************************
** Namespaces
*****************************************************************************/
using namespace std;
namespace guiROS {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
  void init();


	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);

Q_SIGNALS:
    void loggingUpdated();
    void rosShutdown();

// como esta herdando a qthread fazer o override do run para iniciar o desacoplamento
protected:
    void run();

private:
	int init_argc;
	char** init_argv;
  ros::Publisher chatter_publisher;
  QStringListModel logging_model;
};

}  // namespace guiROS

#endif /* guiROS_QNODE_HPP_ */
