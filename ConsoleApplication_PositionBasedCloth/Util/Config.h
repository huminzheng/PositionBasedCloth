#ifndef CONFIG_H
#define CONFIG_H

#include <string>
#include <array>
#include <fstream>

class Config
{
public:
	static std::string const modelPath;
	static std::string const matrixPath;
	static std::string const spherePath;

	template <typename Type, size_t Size>
	static std::array<Type, Size> readData(const char* path)
	{
		std::array<Type, Size> res;
		std::ifstream fin(path);
		for (auto & f : res)
		{
			float fs;
			fin >> fs;
			f = fs;
		}
		fin.close();
		return std::move(res);
	}

private:
	Config() = delete;

};

#endif
