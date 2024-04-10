
#include <iostream>
#include <string>
#include <vector>



class tool_class
{
public:

	/// <summary>
	/// 可视化展示pcd文件
	/// </summary>
	/// <param name="pcdFile">cpd文件路径</param>
	/// <returns></returns>
	static bool openPcd(std::string pcdFile);

	/// <summary>
	/// 复制点云
	/// </summary>
	/// <param name="fromPcd"></param>
	/// <param name="toPcd"></param>
	/// <returns></returns>
	static bool copyPcd(std::string fromPcd, std::string toPcd);

	static bool link(std::string fpcd, std::string spcd);





	tool_class();
	~tool_class();

private:

};







