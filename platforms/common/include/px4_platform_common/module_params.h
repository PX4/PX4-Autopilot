/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

/**
 * @file px4_module_params.h
 *
 * @class ModuleParams is a C++ base class for modules/classes using configuration parameters.
 */

#pragma once

#include <containers/List.hpp>

#include "param.h"
#include <uORB/Subscription.hpp>
#include <uORB/topics/parameter_update.h>

class ModuleParams : public ListNode<ModuleParams *>
{
public:

	ModuleParams(ModuleParams *parent)
	{
		setParent(parent);
	}

	/**
	 * @brief Sets the parent module. This is typically not required,
	 *         only in cases where the parent cannot be set via constructor.
	 */
	void setParent(ModuleParams *parent)
	{
		if (parent) {
			parent->_children.add(this);
		}

		_parent = parent;
	}

	virtual ~ModuleParams()
	{
		if (_parent) { _parent->_children.remove(this); }

		_parameter_update_sub.unsubscribe();
	}

	// Disallow copy construction and move assignment.
	ModuleParams(const ModuleParams &) = delete;
	ModuleParams &operator=(const ModuleParams &) = delete;
	ModuleParams(ModuleParams &&) = delete;
	ModuleParams &operator=(ModuleParams &&) = delete;

protected:
	/**
	 * @brief Call this method whenever the module gets a parameter change notification.
	 *        It will automatically call updateParams() for all children, which then call updateParamsImpl().
	 */
	virtual void updateParams()
	{
		bool update_all = true;
		int parameter_updates = 0;

		while (_parameter_update_sub.updated() && (parameter_updates < parameter_update_s::ORB_QUEUE_LENGTH)) {
			parameter_updates++;

			parameter_update_s parameter_update;

			if (_parameter_update_sub.copy(&parameter_update)
			    && (_parameter_update_instance > 0)
			    && (parameter_update.instance == _parameter_update_instance + 1)
			    && (parameter_update.changed_param.index >= 0)
			    && (static_cast<uint16_t>(parameter_update.changed_param.index) != PARAM_INVALID)
			   ) {
				update_all = false;

				for (const auto &child : _children) {
					child->updateParams(parameter_update);
				}

				updateParamsImpl(parameter_update);

				_parameter_update_instance = parameter_update.instance;

			} else {
				update_all = true;
				break;
			}
		}

		if (update_all) {
			PX4_DEBUG("updateParams: updating all params");
			parameter_update_s parameter_update;

			if (_parameter_update_sub.copy(&parameter_update)) {
				_parameter_update_instance = parameter_update.instance;
			}

			for (const auto &child : _children) {
				child->updateParams();
			}

			updateParamsImpl();

		} else {
			PX4_DEBUG("updateParams: updating all params skipped");
		}
	}

	virtual void updateParams(const parameter_update_s &parameter_update)
	{
		for (const auto &child : _children) {
			child->updateParams(parameter_update);
		}

		updateParamsImpl(parameter_update);
	}

	/**
	 * @brief The implementation for this is generated with the macro DEFINE_PARAMETERS()
	 */
	virtual void updateParamsImpl() {}
	virtual void updateParamsImpl(const parameter_update_s &parameter_update) {}

private:
	/** @list _children The module parameter list of inheriting classes. */
	List<ModuleParams *> _children;
	ModuleParams *_parent{nullptr};

	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uint32_t _parameter_update_instance{0};
};
