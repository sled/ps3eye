/* automatically generated by xdrgen from xdr/geometry_msgs_TransformStamped.xdr
 * xdrgen cvs version: $Revision: 1.7 $ $Date: 2009/01/12 15:54:59 $
 * xdr/geometry_msgs_TransformStamped.xdr last modified: Thu Nov 11 11:15:54 2010
 */

#ifndef INCgeometry_msgs_TransformStamped_h
#define INCgeometry_msgs_TransformStamped_h



#include <ipc_bridge/msgs/roslib_Header.h>
#include <ipc_bridge/msgs/geometry_msgs_Transform.h>


struct geometry_msgs_TransformStamped {
  roslib_Header header;
  char *child_frame_id;
  geometry_msgs_Transform transform;
#define geometry_msgs_TransformStamped_IPC_FORMAT "{" roslib_Header_IPC_FORMAT ",string," geometry_msgs_Transform_IPC_FORMAT "}"
  static const char *getIPCFormat(void) {
    return geometry_msgs_TransformStamped_IPC_FORMAT;
  }


  geometry_msgs_TransformStamped() : child_frame_id(0) {}
  geometry_msgs_TransformStamped(const geometry_msgs_TransformStamped& msg) :
    header(msg.header), transform(msg.transform)
  {
    child_frame_id = new char[strlen(msg.child_frame_id) + 1];
    strcpy(child_frame_id, msg.child_frame_id);
  }
  ~geometry_msgs_TransformStamped()
  {
    if (child_frame_id != 0)
      delete[] child_frame_id;
  }
  geometry_msgs_TransformStamped& operator= (const geometry_msgs_TransformStamped& msg)
  {
    header = msg.header;
    transform = msg.transform;

    if (child_frame_id != 0)
      delete[] child_frame_id;

    child_frame_id = new char[strlen(msg.child_frame_id) + 1];
    strcpy(child_frame_id, msg.child_frame_id);

    return *this;
  }

};


namespace ipc_bridge
{
    namespace geometry_msgs
    {
        typedef geometry_msgs_TransformStamped TransformStamped;
    }
}


#endif /* INCgeometry_msgs_TransformStamped_h */
