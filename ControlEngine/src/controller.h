#pragma once
#include <string>
#include <sstream>
#include <typeinfo>

namespace spim
{
	class Controller
	{
	public:
		inline std::string identify()
		{
			const void * address = static_cast<const void*>(this);
			std::stringstream ss;
			ss << address;  
			return std::string(typeid(*this).name()) + "_" + ss.str();
		};
		virtual std::string details() = 0;

	private:
		
	};
}
