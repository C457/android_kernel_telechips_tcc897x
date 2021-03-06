/*
 * Copyright (c) 2014-2015, The Linux Foundation. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#undef TRACE_SYSTEM
#define TRACE_SYSTEM clk

#if !defined(_TRACE_CLK_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_CLK_H

#include <linux/tracepoint.h>

struct clk;

DECLARE_EVENT_CLASS(clk,

	TP_PROTO(struct clk *core),

	TP_ARGS(core),

	TP_STRUCT__entry(
		__string(        name,           core->name       )
	),

	TP_fast_assign(
		__assign_str(name, core->name);
	),

	TP_printk("%s", __get_str(name))
);

DEFINE_EVENT(clk, clk_enable,

	TP_PROTO(struct clk *core),

	TP_ARGS(core)
);

DEFINE_EVENT(clk, clk_enable_complete,

	TP_PROTO(struct clk *core),

	TP_ARGS(core)
);

DEFINE_EVENT(clk, clk_disable,

	TP_PROTO(struct clk *core),

	TP_ARGS(core)
);

DEFINE_EVENT(clk, clk_disable_complete,

	TP_PROTO(struct clk *core),

	TP_ARGS(core)
);

DEFINE_EVENT(clk, clk_prepare,

	TP_PROTO(struct clk *core),

	TP_ARGS(core)
);

DEFINE_EVENT(clk, clk_prepare_complete,

	TP_PROTO(struct clk *core),

	TP_ARGS(core)
);

DEFINE_EVENT(clk, clk_unprepare,

	TP_PROTO(struct clk *core),

	TP_ARGS(core)
);

DEFINE_EVENT(clk, clk_unprepare_complete,

	TP_PROTO(struct clk *core),

	TP_ARGS(core)
);

DECLARE_EVENT_CLASS(clk_rate,

	TP_PROTO(struct clk *core, unsigned long rate),

	TP_ARGS(core, rate),

	TP_STRUCT__entry(
		__string(        name,           core->name                )
		__field(unsigned long,           rate                      )
	),

	TP_fast_assign(
		__assign_str(name, core->name);
		__entry->rate = rate;
	),

	TP_printk("%s %lu", __get_str(name), (unsigned long)__entry->rate)
);

DEFINE_EVENT(clk_rate, clk_set_rate,

	TP_PROTO(struct clk *core, unsigned long rate),

	TP_ARGS(core, rate)
);

DEFINE_EVENT(clk_rate, clk_set_rate_complete,

	TP_PROTO(struct clk *core, unsigned long rate),

	TP_ARGS(core, rate)
);

DECLARE_EVENT_CLASS(clk_parent,

	TP_PROTO(struct clk *core, struct clk *parent),

	TP_ARGS(core, parent),

	TP_STRUCT__entry(
		__string(        name,           core->name                )
		__string(        pname,          parent->name              )
	),

	TP_fast_assign(
		__assign_str(name, core->name);
		__assign_str(pname, parent->name);
	),

	TP_printk("%s %s", __get_str(name), __get_str(pname))
);

DEFINE_EVENT(clk_parent, clk_set_parent,

	TP_PROTO(struct clk *core, struct clk *parent),

	TP_ARGS(core, parent)
);

DEFINE_EVENT(clk_parent, clk_set_parent_complete,

	TP_PROTO(struct clk *core, struct clk *parent),

	TP_ARGS(core, parent)
);

DECLARE_EVENT_CLASS(clk_phase,

	TP_PROTO(struct clk *core, int phase),

	TP_ARGS(core, phase),

	TP_STRUCT__entry(
		__string(        name,           core->name                )
		__field(	  int,           phase                     )
	),

	TP_fast_assign(
		__assign_str(name, core->name);
		__entry->phase = phase;
	),

	TP_printk("%s %d", __get_str(name), (int)__entry->phase)
);

DEFINE_EVENT(clk_phase, clk_set_phase,

	TP_PROTO(struct clk *core, int phase),

	TP_ARGS(core, phase)
);

DEFINE_EVENT(clk_phase, clk_set_phase_complete,

	TP_PROTO(struct clk *core, int phase),

	TP_ARGS(core, phase)
);

#endif /* _TRACE_CLK_H */

/* This part must be outside protection */
#include <trace/define_trace.h>
