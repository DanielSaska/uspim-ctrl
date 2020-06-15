#pragma once
#include <string>

namespace spim
{
	/// Simple pure virtual class representing an entity that is able to accept string commands
	class InstructionConsumer
	{
	public:
		/// Accepts an instruction, returns true if the instruciton was successfully consumed (executed)
		/// or false otherwise
		virtual bool consumeInstruction(std::string instruction) = 0;
	};
}