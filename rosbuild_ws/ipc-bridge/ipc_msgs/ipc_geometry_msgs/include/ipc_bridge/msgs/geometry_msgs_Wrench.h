/* automatically generated by xdrgen from xdr/geometry_msgs_Wrench.xdr
 * xdrgen cvs version: $Revision: 1.7 $ $Date: 2009/01/12 15:54:59 $
 * xdr/geometry_msgs_Wrench.xdr last modified: Thu Nov 11 11:15:54 2010
 */

#ifndef INCgeometry_msgs_Wrench_h
#define INCgeometry_msgs_Wrench_h



#include <ipc_bridge/msgs/geometry_msgs_Vector3.h>


struct geometry_msgs_Wrench {
  geometry_msgs_Vector3 force;
  geometry_msgs_Vector3 torque;
#define geometry_msgs_Wrench_IPC_FORMAT "{" geometry_msgs_Vector3_IPC_FORMAT "," geometry_msgs_Vector3_IPC_FORMAT "}"
  static const char *getIPCFormat(void) {
    return geometry_msgs_Wrench_IPC_FORMAT;
  }
};


namespace ipc_bridge
{
    namespace geometry_msgs
    {
        typedef geometry_msgs_Wrench Wrench;
    }
}


#endif /* INCgeometry_msgs_Wrench_h */
