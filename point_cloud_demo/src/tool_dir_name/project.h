
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


	static bool copyPcd(std::string fromPcd, std::string toPcd);







	tool_class();
	~tool_class();

private:

};







