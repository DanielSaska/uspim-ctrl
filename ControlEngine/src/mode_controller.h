#pragma once
#include <mutex>
#include "controller.h"

namespace spim
{
	class ModeController : public Controller
	{
	public:
		virtual std::string details() = 0;
	protected:
		//void _l() { _mtx.lock(); };
		//void _u() { _mtx.unlock(); };
	private:
		//std::mutex _mtx;
	};
}
