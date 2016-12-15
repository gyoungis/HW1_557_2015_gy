#include "CheckOutcome.h"
#ifdef ACISOBJ
outcome spa::CheckOutcome::s_lastOutcome;
std::auto_ptr<spa::CheckOutcome::Logger> spa::CheckOutcome::s_logger;
std::auto_ptr<spa::CheckOutcome::Progress> spa::CheckOutcome::s_progress;

// static
outcome const &
spa::CheckOutcome::getLastOutcome(void)
{
	return s_lastOutcome;
}

// static
std::string
spa::CheckOutcome::getLastErrorMessage(void)
{
	err_mess_type err_no = s_lastOutcome.error_number();
	return std::string(find_err_mess(err_no));
}

// static
void spa::CheckOutcome::setLogger(Logger *cb)
{
	s_logger.reset(cb);
}

// static
void spa::CheckOutcome::setProgress(Progress *cb)
{
	s_progress.reset(cb);
}

// static
bool
spa::CheckOutcome::pre(std::string const &progressMessage)
{
	static bool progressCallbackInitialized = false;
	if (!progressCallbackInitialized) {
		set_progress_callback(progressCallback);
		progressCallbackInitialized = true;
	}

	if (NULL != s_progress.get()) {
		s_progress->willCallApi(progressMessage);
	}

	return true;
}

// static
bool
spa::CheckOutcome::post(void)
{
	if (NULL != s_progress.get()) {
		s_progress->didCallApi(s_lastOutcome);
	}
	return s_lastOutcome.ok();
}

// static
bool
spa::CheckOutcome::checkOutcome_internal(char const *api, char const *sourceFile, int const sourceLine, outcome result)
{
	s_lastOutcome = result;
	if (NULL != s_logger.get()) {
		s_logger->log(api, sourceFile, sourceLine, result);
	}
	return true;
}
// static
int spa::CheckOutcome::progressCallback(SPA_progress_info *pi)
{
	if (NULL != s_progress.get()) {
		bool ok = s_progress->didUpdateProgress(pi->percentage());
		if (!ok) {
			interrupt_acis();
			return -1;
		}
	}
	return 0;
}
#endif