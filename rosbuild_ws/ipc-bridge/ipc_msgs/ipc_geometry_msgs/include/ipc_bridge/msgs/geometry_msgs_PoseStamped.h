/* automatically generated by xdrgen from xdr/geometry_msgs_PoseStamped.xdr
 * xdrgen cvs version: $Revision: 1.7 $ $Date: 2009/01/12 15:54:59 $
 * xdr/geometry_msgs_PoseStamped.xdr last modified: Thu Nov 11 11:15:54 2010
 */

#ifndef INCgeometry_msgs_PoseStamped_h
#define INCgeometry_msgs_PoseStamped_h



#include <ipc_bridge/msgs/roslib_Header.h>
#include <ipc_bridge/msgs/geometry_msgs_Pose.h>


struct geometry_msgs_PoseStamped {
  roslib_Header header;
  geometry_msgs_Pose pose;
#define geometry_msgs_PoseStamped_IPC_FORMAT "{" roslib_Header_IPC_FORMAT "," geometry_msgs_Pose_IPC_FORMAT "}"
  static const char *getIPCFormat(void) {
    return geometry_msgs_PoseStamped_IPC_FORMAT;
  }
};


namespace ipc_bridge
{
    namespace geometry_msgs
    {
        typedef geometry_msgs_PoseStamped PoseStamped;
    }
}


#endif /* INCgeometry_msgs_PoseStamped_h */