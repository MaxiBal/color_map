#include <color_map/color.hpp>

namespace color_map
{

// Define ColorHandler Implementation class

class ColorHandler::ColorHandlerImpl
{

	float scale_temp_data();
	float calc_r();
	float calc_g();
	float calc_b();

	friend class ColorHandler;

public:
	ColorHandlerImpl() = default;

	ColorHandler::Colors get_rgb();
};


float ColorHandler::ColorHandlerImpl::scale_temp_data(float temp)
{
	// min max standardizing

	return (temp - est_temp_min) / est_temp_range;
}

float ColorHandler::ColorHandlerImpl::calc_r(float temp)
{

	return clamp(
		2.22 * temp - 1.74,
		0, 255
	);
}

float ColorHandler::ColorHandlerImpl::calc_g(float temp)
{
	return clamp(-1.17 * x + 213, 0, 255);
}

float ColorHandler::ColorHandlerImpl::calc_b(float temp)
{

	return clamp(
		-2.55 * temp + 344,
		0, 255
	);
}

ColorHandler::Colors ColorHandler::ColorHandlerImpl::get_rgb(float temp)
{
	float scaled_temp = scale_temp_data(temp);

	return {calc_r(scaled_temp), calc_g(scaled_temp), calc_b(scaled_temp)};
}

// End Define ColorHandlerImpl


// ColorHandler constructor
// initialize ColorHandler implementation pointer

ColorHandler::ColorHandler(float temp) : pImpl(ColorHandlerImpl()), raw_temp(temp) {}

// unpack values given from 

std::tuple<float, float, float> ColorHandler::get_rgb()
{
	const auto [r, g, b] = pImpl->get_rgb(raw_temp);

	return {r, g, b};
}