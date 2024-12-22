#ifndef CHANNELDEVICE_H_
#define CHANNELDEVICE_H_

#include <core/base/v8stdint.h>
namespace ordlidar {
namespace core {
namespace common {
class ChannelDevice {
 public:
  ChannelDevice() {}
  virtual ~ChannelDevice() {}
  /**
   * @brief bind device port
   * @return true if successfully bind, otherwise false
   */
  virtual bool bindport(const char *, uint32_t) {
    return true;
  }
  /**
   * @brief open device
   * @return true if successfully open, otherwise false
   */
  virtual bool open() = 0;
  /**
   * @brief Whether is open
   * @return true if already open, otherwise false
   */
  virtual bool isOpen() = 0;
  /**
   * @brief close serial port or network
   */
  virtual void closePort() = 0;
  /**
   * @brief Return the number of characters in the buffer.
   * @return
   */
  virtual size_t available() = 0;


  /*!
   * @brief Flush the input and output buffers
   */
  virtual void flush() = 0;
  /**
   * @brief Block until there is serial or network data to read or read_timeout_constant
   * number of milliseconds have elapsed. The return value is greater than zero when
   * the function exits with the serial port or network buffer is greater than or
   * equal to data_count, false otherwise(due to timeout or select interruption).
   * @param data_count A size_t that indicates how many bytes should be wait from
   * the given serial port or network buffer.
   * @param timeout waiting timeout time
   * @param returned_size if it is not NULL, the actual number of bytes will be returned.
   * @return A size_t representing the number of bytes wait as a result of the
   * call to wait.
   */
  virtual int waitfordata(size_t data_count, uint32_t timeout = -1,
                          size_t *returned_size = NULL) = 0;
  /*! Read a given amount of bytes from the serial port or network and return a string
  *  containing the data.
  *
  * \param size A size_t defining how many bytes to be read.
  *
  * \return A std::string containing the data read from the port.
  *
  */
  virtual std::string readSize(size_t size = 1) = 0;

  /*! Write a string to the serial port or network.
  *
  * \param data A const reference containing the data to be written
  * to the serial port.
  *
  * \param size A size_t that indicates how many bytes should be written from
  * the given data buffer.
  *
  * \return A size_t representing the number of bytes actually written to
  * the serial port.
  */
  virtual size_t writeData(const uint8_t *data, size_t size) = 0;

  /*! Read a given amount of bytes from the serial port or network into a given buffer.
  *
  * The read function will return in one of three cases:
  *  * The number of requested bytes was read.
  *    * In this case the number of bytes requested will match the size_t
  *      returned by read.
  *  * A timeout occurred, in this case the number of bytes read will not
  *    match the amount requested, but no exception will be thrown.  One of
  *    two possible timeouts occurred:
  *    * The inter byte timeout expired, this means that number of
  *      milliseconds elapsed between receiving bytes from the serial port
  *      exceeded the inter byte timeout.
  *    * The total timeout expired, which is calculated by multiplying the
  *      read timeout multiplier by the number of requested bytes and then
  *      added to the read timeout constant.  If that total number of
  *      milliseconds elapses after the initial call to read a timeout will
  *      occur.
  *  * An exception occurred, in this case an actual exception will be thrown.
  *
  * \param buffer An uint8_t array of at least the requested size.
  * \param size A size_t defining how many bytes to be read.
  *
  * \return A size_t representing the number of bytes read as a result of the
  *         call to read.
  *
  */
  virtual size_t readData(uint8_t *data, size_t size) = 0;
  /*!
   * @brief Set the DTR handshaking line to the given level.
   * @param level Defaults to true.
   */

  virtual bool setDTR(bool level = true) {
    return true;
  }
  /*!
   * @brief Returns the singal byte time.
   * @return one byte transfer time
   */
  virtual int getByteTime() {
    return 0;
  }

  /**
   * @brief Returns a human-readable description of the given error code
   *  or the last error code of a socket or serial port
   * @return error information
   */
  virtual const char *DescribeError() {
    return "";
  }

};
}//common
}//core
}//ordlidar

#endif
