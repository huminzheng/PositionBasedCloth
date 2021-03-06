#ifndef CONFIG_H
#define CONFIG_H

#include <rapidjson\rapidjson.h>
#include <rapidjson\document.h>
#include <rapidjson\writer.h>
#include <rapidjson\stringbuffer.h>
#include <rapidjson\filereadstream.h>
#include <rapidjson\pointer.h>

#include <string>
#include <array>
#include <fstream>

class PBDParams
{
public:
	PBDParams() = default;

	float YoungModulo_xx;
	float YoungModulo_yy;
	float YoungModulo_xy;
	float PoissonRation_xy;
	float PoissonRation_yx;
	float IsometricBendingStiff;
	float ClothThickness;

	float ProjectDistanceStiff;
	float ProjectIsometricBendingStiff;
	float ProjectFEMTriangleStiff;

	float IterCount;

};

class Config
{
public:
	std::string modelPath;
	std::string matrixPath;
	std::string rigidBodyPath;

	PBDParams * parameters;

	Config(const char * path)
	{
		load(path);
	}

private:


	bool load(const char * path)
	{
		FILE* fp = fopen(path, "rb"); // 非Windows平台使用"r"
		if (fp == nullptr)
			return false;
		
		char readBuffer[65536];
		rapidjson::FileReadStream is(fp, readBuffer, sizeof(readBuffer));
		rapidjson::Document doc;
		doc.ParseStream(is);
		fclose(fp);

		auto const & root = doc.GetObject();
		
		/* get config params */
		auto const & entry = root.FindMember("PathGroup")->value.GetString();
		auto const & dirs = root.FindMember(entry)->value.GetObject();

		this->matrixPath = dirs.FindMember("ClothMatrixPath")->value.GetString();
		this->rigidBodyPath = dirs.FindMember("RigidBodyModelPath")->value.GetString();
		this->modelPath = dirs.FindMember("ClothModelPath")->value.GetString();

		/* get model params */
		auto const & model = root.FindMember("ParamGroup")->value.GetString();
		auto const & params = root.FindMember(model)->value.GetObject();

		this->parameters = new PBDParams();
		this->parameters->YoungModulo_xx = params.FindMember("YoungModulo_xx")->value.GetFloat();
		this->parameters->YoungModulo_yy = params.FindMember("YoungModulo_yy")->value.GetFloat();
		this->parameters->YoungModulo_xy = params.FindMember("YoungModulo_xy")->value.GetFloat();

		this->parameters->PoissonRation_xy = params.FindMember("PoissonRation_xy")->value.GetFloat();
		this->parameters->PoissonRation_yx = params.FindMember("PoissonRation_yx")->value.GetFloat(); 
		
		this->parameters->IsometricBendingStiff = params.FindMember("IsometricBendingStiff")->value.GetFloat();
		
		this->parameters->ClothThickness = params.FindMember("ClothThickness")->value.GetFloat();
		
		this->parameters->ProjectDistanceStiff = params.FindMember("ProjectDistanceStiff")->value.GetFloat();
		this->parameters->ProjectIsometricBendingStiff = params.FindMember("ProjectIsometricBendingStiff")->value.GetFloat();
		this->parameters->ProjectFEMTriangleStiff = params.FindMember("ProjectFEMTriangleStiff")->value.GetFloat();

		this->parameters->IterCount = params.FindMember("FrameIterationCount")->value.GetInt();

	}
	
};


template <typename Type, size_t Size>
std::array<Type, Size> readData(const char* path)
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


#endif
