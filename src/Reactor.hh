// $Id$

#ifndef REACTOR_HH
#define REACTOR_HH

#include "Observer.hh"
#include "EventListener.hh"
#include <memory>

namespace openmsx {

class CommandController;
class EventDistributor;
class CommandConsole;
class FileManipulator;
class FilePool;
class CliComm;
class BooleanSetting;
class MSXMotherBoard;
class Setting;
class CommandLineParser;
class QuitCommand;
class ExitCPULoopSchedulable;

/**
 * Contains the main loop of openMSX.
 * openMSX is almost single threaded: the main thread does most of the work,
 * we create additional threads only if we need blocking calls for
 * communicating with peripherals.
 * This class serializes all incoming requests so they can be handled by the
 * main thread.
 */
class Reactor : private Observer<Setting>, private EventListener
{
public:
	Reactor();
	~Reactor();

	/**
	 * Main loop.
	 * @param autoRun Iff true, start emulation immediately.
	 */
	void run(CommandLineParser& parser);

	void enterMainLoop();

	CommandController& getCommandController();
	EventDistributor& getEventDistributor();
	CommandConsole& getCommandConsole();
	FileManipulator& getFileManipulator();
	FilePool& getFilePool();
	MSXMotherBoard& getMotherBoard();

private:
	// Observer<Setting>
	virtual void update(const Setting& setting);

	// EventListener
	virtual void signalEvent(const Event& event);

	void block();
	void unblock();

	void unpause();
	void pause();

	bool paused;

	int blockedCounter;

	/**
	 * True iff the Reactor should keep running.
	 * When this is set to false, the Reactor will end the main loop after
	 * finishing the pending request(s).
	 */
	bool running;

	// note: order of auto_ptr's is important
	std::auto_ptr<CommandController> commandController;
	std::auto_ptr<EventDistributor> eventDistributor;
	std::auto_ptr<CommandConsole> commandConsole;
	std::auto_ptr<FileManipulator> fileManipulator;
	std::auto_ptr<FilePool> filePool;
	std::auto_ptr<MSXMotherBoard> motherBoard;

	BooleanSetting& pauseSetting;
	CliComm& cliComm;
	const std::auto_ptr<QuitCommand> quitCommand;
	friend class QuitCommand;

	std::auto_ptr<ExitCPULoopSchedulable> schedulable;
};

} // namespace openmsx

#endif // REACTOR_HH
