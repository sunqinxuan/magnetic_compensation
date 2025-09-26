#include <GeoMag/Core.hpp>

using namespace geomag;

int main(int argc, char** argv) {
	DateTime date;
	double lat, lon, alt;

	if (argc == 5) {
		try {
			date = DateTime(argv[1]);
			lat = std::stod(argv[2]);
			lon = std::stod(argv[3]);
			alt = std::stod(argv[4]);
		} catch (std::exception& e) {
			std::cout << "Format Error: " << e.what() << std::endl;
			return 1;
		}
	} else {
		std::cout << "Usage: " << argv[0] << " date lat lon alt" << std::endl;
		return 1;
	}

	auto gmag = GeoMagFlux{MagFluxUnit::NanoTesla};
	auto position = Wgs84{date, Degree{lon}, Degree{lat}, alt};
	auto bf = gmag(position);
	auto b = MagFluxComponent{bf};

	std::cout << "Position: " << position << "\n";
	std::cout << "Mag flux: " << b.north << " " << b.east << " " << b.down << " " << b.total << " " << b.horizontal << " " << b.inclination << " "
			  << b.declination << std::endl;
}
