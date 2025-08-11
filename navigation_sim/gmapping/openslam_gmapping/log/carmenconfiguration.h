#ifndef CARMENCONFIGURATION_H
#define CARMENCONFIGURATION_H

#include <string>
#include <map>
#include <vector>
#include <istream>
#include <sensor/sensor_base/sensor.h>
#include "configuration.h"

namespace GMapping {

class CarmenConfiguration: public Configuration, public std::map<std::string, std::vector<std::string> >{
	public:
		virtual std::istream& load(std::istream& is);
		virtual SensorMap computeSensorMap() const;
};

};

#endif

