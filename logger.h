//
// Created by etienne on 03.11.20.
//

#ifndef MCSAMPLING_LOGGER_H
#define MCSAMPLING_LOGGER_H

#endif //MCSAMPLING_LOGGER_H

#include <iostream>
#include <fstream>
#include <stdexcept>

class Logger {
public:
	// Logger cannot exist without file.
	Logger() = delete;

	// Disable copy constructor  since std::ofstream is not copyable.
	Logger(Logger const&) = delete;

	// Constructor
	explicit Logger(std::string const& f_path)
			: log_file { f_path }
	{
		if (!log_file.is_open())
		{
			throw std::runtime_error("Unable to open log file");
		}
	}

	// Disable copy.
	Logger& operator=(Logger const&) = delete;

	// Cleanup.
	~Logger()
	{
		log_file.close();
	}

	// Write a single value into log file stream.
	template<typename T>
	void write(T const& v)
	{
		log_file << v << ",";
	}

	void write_endl()
	{
		log_file << "\n";
	}

	// Write multiple values.
	template<typename Arg, typename ...Args>
	void write(Arg const& arg, Args const&... args)
	{
		// here we write the first value of the values list.
		write(arg);
		// here we recursively pass the rest values to the function.
		write(args...);
	}
private:
	// Log file stream.
	std::ofstream log_file;
};