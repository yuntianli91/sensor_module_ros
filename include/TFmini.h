/*
 * @Description: 
 * @Author: Yuntian Li
 * @Github: https://github.com/yuntinali91
 * @Date: 2019-11-05 20:26:15
 * @LastEditors: Yuntian Li
 * @LastEditTime: 2019-11-05 20:26:16
 */
#include "common_headers.h"

namespace benewake
{
  class TFmini
  {
    public:
      TFmini(std::string _name, int _baudRate);
      ~TFmini(){};
      float getDist();
      void closePort();

      unsigned char dataBuf[7];

    private:
      std::string portName_;
      int baudRate_;
      int serial_;

      bool readData(unsigned char *_buf, int _nRead);
  };
}
