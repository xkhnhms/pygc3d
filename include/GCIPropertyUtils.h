#pragma once
#include <string.h>
#include <string>
namespace GCIPropertyUtils
{
	struct ConfigFile;
	/**
	* 打开类似以下文件格式的配置文件
	* #server ip port
	* server.ip = 127.0.0.1
	* server.port = 8080
	*/
	ConfigFile* OpenConfigFile(const char* filepath, bool bReadonly = true);
	/**
	* 保存配置
	*/
	void Save(ConfigFile* cf);
	/**
	* 关闭配置, 如果配置有发生变化,则自动保存
	*/
	void Close(ConfigFile* cf);

	std::string GetPropertyString(ConfigFile* cf, const char* key, const char* defaultV = "");
	bool GetPropertyBool(ConfigFile* cf, const char* key, bool defaultV = false);
	int GetPropertyInteger(ConfigFile* cf, const char* key, int defaultV = 0);
	void WriteProperty(ConfigFile* cf, const char* key, const char* v);
	void WriteProperty(ConfigFile* cf, const char* key, bool v);
	void WriteProperty(ConfigFile* cf, const char* key, int v);
    void WriteProperty(ConfigFile* cf, const char* key, float v);
    void WriteProperty(ConfigFile* cf, const char* key, double v);
}
