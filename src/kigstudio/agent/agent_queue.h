#pragma once

/**
 * Thread-safe command queue bridging HTTP worker threads and the main
 * render thread.  HTTP handlers push AgentCommand structs; the main
 * loop drains them once per frame via process_commands().
 *
 * All state changes happen on the main thread — the HTTP threads only
 * parse JSON and format responses.  This avoids locking contention
 * with the render loop.
 */

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <future>
#include <mutex>
#include <string>

#include <cJSON.h>

namespace sinriv::kigstudio::agent {

/// One pending agent command travelling from an HTTP worker to the main thread.
struct AgentCommand {
	std::string method;         // e.g. "node.get", "system.status"
	cJSON* params = nullptr;    // request body as cJSON (owned; freed by main thread)
	std::promise<cJSON*> result_promise;  // fulfilled by main thread
	std::uint32_t timeout_ms = 30000;
};

/// Bounded, thread-safe queue of AgentCommand with condition-variable
/// wake-up for the main thread.
class AgentCommandQueue {
public:
	static constexpr size_t kMaxQueueSize = 256;

	/// Push a command from an HTTP worker thread.  Blocks if the queue
	/// is full (back-pressure to prevent memory exhaustion under load).
	void push(AgentCommand&& cmd) {
		std::unique_lock<std::mutex> lk(mutex_);
		// Wait with a short timeout so we don't deadlock if the main
		// thread is stuck.
		while (queue_.size() >= kMaxQueueSize) {
			if (cv_full_.wait_for(lk, std::chrono::milliseconds(500)) ==
			    std::cv_status::timeout) {
				// Drop oldest command as emergency back-pressure relief
				if (!queue_.empty()) {
					if (queue_.front().params)
						cJSON_Delete(queue_.front().params);
					queue_.front().result_promise.set_value(nullptr);
					queue_.pop_front();
				}
			}
		}
		queue_.push_back(std::move(cmd));
		cv_.notify_one();
	}

	/// Called by the main thread each frame.  Processes ALL pending
	/// commands against `list`.  Each command's promise is fulfilled
	/// so the waiting HTTP thread can respond.
	///
	/// `handler` is a callable with signature:
	///   cJSON*(const std::string& method, cJSON* params, RenderVoxelList& list)
	///
	/// The handler takes ownership of `params` and the return value
	/// ownership is transferred to the promise.
	template <typename List, typename Handler>
	void process_commands(List& list, Handler&& handler) {
		// Fast-path check before acquiring the mutex
		{
			std::lock_guard<std::mutex> lk(mutex_);
			if (queue_.empty())
				return;
		}

		// Drain everything currently queued
		std::deque<AgentCommand> batch;
		{
			std::lock_guard<std::mutex> lk(mutex_);
			batch.swap(queue_);
			cv_full_.notify_all();
		}

		for (auto& cmd : batch) {
			cJSON* result = nullptr;
			try {
				result = handler(cmd.method, cmd.params, list);
			} catch (...) {
				// On exception, return error JSON so the HTTP client
				// doesn't hang forever.
			}
			// params ownership transferred to handler; handler must free it.
			// Return nullptr on internal error.
			if (!result) {
				result = cJSON_CreateObject();
				cJSON_AddFalseToObject(result, "ok");
				cJSON_AddStringToObject(result, "error", "internal error");
			}
			cmd.result_promise.set_value(result);
		}
	}

	/// Drain all pending commands, resolving each promise to nullptr.
	/// Used during shutdown to unblock any waiting HTTP/MCP threads.
	void drain() {
		std::lock_guard<std::mutex> lk(mutex_);
		for (auto& cmd : queue_) {
			if (cmd.params)
				cJSON_Delete(cmd.params);
			cmd.result_promise.set_value(nullptr);
		}
		queue_.clear();
		cv_full_.notify_all();
	}

	/// Non-blocking check — true when the queue is empty.
	bool empty() const {
		std::lock_guard<std::mutex> lk(mutex_);
		return queue_.empty();
	}

	size_t size() const {
		std::lock_guard<std::mutex> lk(mutex_);
		return queue_.size();
	}

private:
	mutable std::mutex mutex_;
	std::condition_variable cv_;
	std::condition_variable cv_full_;
	std::deque<AgentCommand> queue_;
};

}  // namespace sinriv::kigstudio::agent
