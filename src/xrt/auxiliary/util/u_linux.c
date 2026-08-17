// Copyright 2023, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Various helpers for doing Linux specific things.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 *
 * @ingroup aux_util
 */

#include "util/u_linux.h"
#include "util/u_pretty_print.h"

#include <pthread.h>
#include <assert.h>
#include <unistd.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>

#define LOG_D(...) U_LOG_IFL_D(log_level, __VA_ARGS__)
#define LOG_I(...) U_LOG_IFL_I(log_level, __VA_ARGS__)
#define LOG_W(...) U_LOG_IFL_W(log_level, __VA_ARGS__)
#define LOG_E(...) U_LOG_IFL_E(log_level, __VA_ARGS__)

#define NAME_LENGTH 32


/*
 *
 * Helper functions.
 *
 */

static const char *
policy_to_string(int policy)
{
	switch (policy) {
	case SCHED_FIFO: return "SCHED_FIFO";
	case SCHED_RR: return "SCHED_RR";
	case SCHED_OTHER: return "SCHED_OTHER(normal)";
	case SCHED_IDLE: return "SCHED_IDLE";
	case SCHED_BATCH: return "SCHED_BATCH";
	default: return "SCHED_<UNKNOWN>";
	}
}

static void
get_name(char *str, size_t count)
{
	assert(str != NULL);
	assert(count > 0);

	// First init.
	str[0] = '\0';

	// Get name of thread.
	pthread_t this_thread = pthread_self();
	pthread_getname_np(this_thread, str, count);

	if (str[0] == '\0') {
		snprintf(str, count, "tid(%i)", gettid());
	}
}

static void
print_thread_info(struct u_pp_delegate dg, enum u_logging_level log_level, pthread_t thread)
{
	struct sched_param params;
	int policy = 0;
	int ret = 0;

	// Get the policy and scheduling priority.
	ret = pthread_getschedparam(thread, &policy, &params);
	if (ret != 0) {
		LOG_E("pthread_getschedparam: %i", ret);
		return;
	}

	u_pp(dg, "policy: '%s', priority: '%i'", policy_to_string(policy), params.sched_priority);
}


/*
 *
 * 'Exported' functions.
 *
 */

bool
u_linux_try_to_set_realtime_priority_on_thread(enum u_logging_level log_level, const char *name)
{
	pthread_t this_thread = pthread_self();
	struct u_pp_sink_stack_only sink;
	struct sched_param params;
	char str[NAME_LENGTH];
	int ret;

	// Add printing delegate.
	struct u_pp_delegate dg = u_pp_sink_stack_only_init(&sink);

	// Always have some name.
	if (name == NULL) {
		get_name(str, ARRAY_SIZE(str));
		name = str;
	}

	if (log_level <= U_LOGGING_DEBUG) {
		u_pp(dg, "Trying to raise priority on thread '%s'\n\t", name);
		u_pp(dg, "before: ");
		print_thread_info(dg, log_level, this_thread);
	}

	// Get the maximum on this platform.
	params.sched_priority = sched_get_priority_max(SCHED_FIFO);

	// Here we try to set the realtime scheduling with the max priority available.
	ret = pthread_setschedparam(this_thread, SCHED_FIFO, &params);

	// Print different amount depending on log level.
	if (log_level <= U_LOGGING_DEBUG) {
		u_pp(dg, "after: ");
		print_thread_info(dg, log_level, this_thread);
		u_pp(dg, "\n\tResult: %i", ret);
	} else {
		if (ret != 0) {
			u_pp(dg, "Could not raise priority for thread '%s'", name);
		} else {
			u_pp(dg, "Raised priority of thread '%s' to ", name);
			print_thread_info(dg, log_level, this_thread);
		}
	}

	// Always print as warning or information.
	if (ret != 0) {
		LOG_W("%s", sink.buffer);
	} else {
		LOG_I("%s", sink.buffer);
	}

	return ret == 0;
}

/*!
 * Pin this thread to a set of CPUs, if @p env_var_name is set in the environment. Written
 * for the round-2 item T204 left open: "compositor thread priority/affinity under
 * tracking load" -- every Monado-owned compositor thread already runs SCHED_FIFO max
 * priority (see u_linux_try_to_set_realtime_priority_on_thread and its call sites), so
 * more priority has nothing left to buy; separating WHICH cores the compositor and the
 * WMR/tracking side land on is the other half of "scheduling", untried before this. A
 * strict no-op with the env var unset: no syscall, no behaviour change on the healthy path.
 *
 * @p env_var_name's value is a comma-separated list of CPU ids, e.g. "0,1". Unset or empty
 * leaves the thread's affinity exactly as the OS gave it.
 */
bool
u_linux_try_to_set_thread_affinity_from_env(enum u_logging_level log_level, const char *name, const char *env_var_name)
{
	pthread_t this_thread = pthread_self();
	char str[NAME_LENGTH];

	if (name == NULL) {
		get_name(str, ARRAY_SIZE(str));
		name = str;
	}

	const char *list = getenv(env_var_name);
	if (list == NULL || list[0] == '\0') {
		return true;
	}

	cpu_set_t set;
	CPU_ZERO(&set);

	int count = 0;
	const char *p = list;
	while (*p != '\0') {
		char *end = NULL;
		long cpu = strtol(p, &end, 10);
		if (end == p) {
			// Not a number where one was expected: stop parsing rather than guess
			// at what the rest of a malformed value might mean.
			break;
		}
		if (cpu >= 0 && cpu < CPU_SETSIZE) {
			CPU_SET((int)cpu, &set);
			count++;
		}
		p = end;
		while (*p == ',' || *p == ' ') {
			p++;
		}
	}

	if (count == 0) {
		LOG_W("%s='%s' did not parse to any valid CPU id, leaving '%s' unpinned", env_var_name, list, name);
		return false;
	}

	int ret = pthread_setaffinity_np(this_thread, sizeof(set), &set);
	if (ret != 0) {
		LOG_W("Could not pin thread '%s' to CPU set '%s' (%s), errno-style ret=%i", name, list, env_var_name,
		      ret);
	} else {
		LOG_I("Pinned thread '%s' to CPU set '%s' (%s)", name, list, env_var_name);
	}

	return ret == 0;
}
