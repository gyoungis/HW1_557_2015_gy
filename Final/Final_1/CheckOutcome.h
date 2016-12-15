#ifndef spa_CheckOutcome_header_included
#define spa_CheckOutcome_header_included
#include "Includes.h"
#ifdef ACISOBJ
class SPA_progress_info;
namespace spa
{
	class CheckOutcome
	{
	public:
		/*! \brief Returns the outcome object from the last API
		* call passed to the checkOutcome macro. Use this method
		* when you want to obtain more information in the event
		* an API fails (checkOutcome returns false).
		*/
		static outcome const &getLastOutcome(void);
		static std::string getLastErrorMessage(void);

		/*! \name API Call Logging Interface */
		//@{
		/*!
		\interface Logger
		\brief An implementation of this interface can be used with spa::CheckOutcome to capture information
		about ACIS API calls.
		*/
		class Logger
		{
		public:
			Logger() {}
			virtual ~Logger() {}
			virtual void log(std::string const &apiCalled, std::string const &calledFromFilename, int const &calledFromLineNumber, outcome const &result) {
				(void)apiCalled; (void)calledFromFilename; (void)calledFromLineNumber; (void)result;
			}
		};
		/*!
		\brief Sets the progress callback interface object. Takes ownership of object.
		*/
		static void setLogger(Logger *cb);
		//@}

		/*! \name Progress Callback Interface */
		//@{
		/*!
		\interface Progress
		\brief An implementation of this interface can be used with spa::CheckOutcome to capture progress
		information as an ACIS API call is in progress.
		*/
		class Progress
		{
		public:
			Progress() {}
			virtual ~Progress(void) {}
			/*!
			\brief This method is invoke prior to the API call. Often, this method is implemented to
			start a timer for delaying the display of a progress dialog.
			\param message The progress message intended to be shown the the user
			*/
			virtual void willCallApi(std::string const &message) { (void)message; }
			/*
			\brief This method is invoked periodically during the process of executing an API call
			\return Implementations of this method should return true to allow processing to continue. Returning false will abort the API.
			*/
			virtual bool didUpdateProgress(double const &percentComplete) { (void)percentComplete; return true; }
			/*!
			\brief This method is invoked after an API call completes. Often, this method is implemented to
			hide a progress dialog.
			*/
			virtual void didCallApi(outcome const &result) { (void)result; }
		};
		/*!
		\brief Sets the progress callback interface object. Takes ownership of object.
		*/
		static void setProgress(Progress *cb);
		//@}

		static bool pre(std::string const &progressMessage);
		static bool checkOutcome_internal(char const *apiCalled,
			char const *calledFromFile,
			int calledFromLineNumber,
			outcome result);
		static bool post(void);

	private:
		static int progressCallback(SPA_progress_info *pi);
		static outcome s_lastOutcome;
		static std::auto_ptr<Logger> s_logger;
		static std::auto_ptr<Progress> s_progress;
	};
}

/*! \brief This macro should be used to wrap api_ function calls, or any other
function that returns an outcome object
*/
#define checkOutcomeWithProgress( api, msg ) \
(spa::CheckOutcome::pre(msg) && \
spa::CheckOutcome::checkOutcome_internal( # api, __FILE__, __LINE__, api ) && \
spa::CheckOutcome::post())
#define checkOutcome( api ) \
checkOutcomeWithProgress( api, std::string() )
#endif
#endif