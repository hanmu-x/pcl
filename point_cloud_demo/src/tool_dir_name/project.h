
#include <iostream>
#include <string>
#include <vector>



class tool_class
{
public:

	/// <summary>
	/// ���ӻ�չʾpcd�ļ�
	/// </summary>
	/// <param name="pcdFile">cpd�ļ�·��</param>
	/// <returns></returns>
	static bool openPcd(std::string pcdFile);

	/// <summary>
	/// ���Ƶ���
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







