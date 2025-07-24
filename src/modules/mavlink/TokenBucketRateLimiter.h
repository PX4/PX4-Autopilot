/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

class TokenBucketRateLimiter
{
private:
	uint64_t _last_token_update{0};
	float _available_tokens{0.0f};      // Currently available tokens (in bytes)
	float _max_bucket_tokens{0.0f};     // Maximum tokens bucket can hold
	float _tokens_per_microsecond{0.0f}; // Token generation rate
	uint32_t _configured_rate_bytes_per_sec{0}; // For debugging/telemetry

	// Statistics
	uint64_t _total_tokens_requested{0};
	uint64_t _total_tokens_granted{0};
	uint64_t _total_tokens_denied{0};

public:
	/**
	 * Configure the rate limiter
	 * @param bytes_per_sec Maximum data rate in bytes per second
	 */
	void configure_rate(uint32_t bytes_per_sec)
	{
		_configured_rate_bytes_per_sec = bytes_per_sec;
		_tokens_per_microsecond = static_cast<float>(bytes_per_sec) / 1e6f;

		// TODO: how large of a burst should we allow? Probably MAVLINK_MAX_PACKET_LEN

		// Bucket size: allow burst of 100ms worth of data
		// This permits parameter sending bursts while maintaining average rate
		// _max_bucket_tokens = static_cast<float>(bytes_per_sec) * 0.1f;


		// Fill bucket with max bandwidth worth of tokens, duhhh. All our traffic is bursty -_-
		_max_bucket_tokens = static_cast<float>(bytes_per_sec);

		// Don't let available tokens exceed new limit
		if (_available_tokens > _max_bucket_tokens) {
			_available_tokens = _max_bucket_tokens;
		}
	}

	/**
	 * Request tokens for transmission
	 * @param bytes Number of bytes to transmit
	 * @param priority Allow overdraft for high priority messages
	 * @return true if tokens were granted
	 */
	bool request_tokens(size_t bytes, bool priority = false)
	{
		replenish_tokens();

		_total_tokens_requested += bytes;

		const float tokens_needed = static_cast<float>(bytes);

		if (priority && _available_tokens < tokens_needed) {
			// Priority messages can overdraft up to 20% of bucket size
			const float max_overdraft = _max_bucket_tokens * 0.2f;

			if (_available_tokens >= -max_overdraft) {
				_available_tokens -= tokens_needed;
				_total_tokens_granted += bytes;
				return true;
			}
		}

		if (_available_tokens >= tokens_needed) {
			_available_tokens -= tokens_needed;
			_total_tokens_granted += bytes;
			return true;
		}

		_total_tokens_denied += bytes;
		return false;
	}


	bool check_tokens_available(size_t bytes) const
	{
		return _available_tokens >= static_cast<float>(bytes);
	}

	float get_available_tokens()
	{
		replenish_tokens();
		return _available_tokens;
	}

	/**
	 * Get percentage of bucket filled
	 * @return 0.0 to 1.0 representing bucket fill level
	 */
	float get_bucket_fill_ratio()
	{
		replenish_tokens();
		return _available_tokens / _max_bucket_tokens;
	}

	struct Statistics {
		uint64_t tokens_requested;
		uint64_t tokens_granted;
		uint64_t tokens_denied;
		float current_tokens;
		float max_tokens;
		float fill_ratio;
		uint32_t configured_rate;
	};

	Statistics get_statistics()
	{
		replenish_tokens();
		return {
			_total_tokens_requested,
			_total_tokens_granted,
			_total_tokens_denied,
			_available_tokens,
			_max_bucket_tokens,
			get_bucket_fill_ratio(),
			_configured_rate_bytes_per_sec
		};
	}

private:
	void replenish_tokens()
	{
		hrt_abstime timestamp = hrt_absolute_time();
		// Handle first call
		if (_last_token_update == 0) {
			_last_token_update = timestamp;
			_available_tokens = _max_bucket_tokens;
			return;
		}

		const float dt_us = static_cast<float>(timestamp - _last_token_update);
		_last_token_update = timestamp;

		// Generate new tokens based on elapsed time
		const float new_tokens = _tokens_per_microsecond * dt_us;
		_available_tokens += new_tokens;

		// Cap at bucket maximum
		if (_available_tokens > _max_bucket_tokens) {
			_available_tokens = _max_bucket_tokens;
		}
	}
};
