#include <stdio.h>
#include <xtables.h>
#include <linux/netfilter/xt_FLOWOFFLOAD.h>

enum {
    O_HW,
};

static void offload_help(void)
{
	printf(
"FLOWOFFLOAD target options:\n"
" --hw				Enable hardware offload\n"
	);
}

static const struct xt_option_entry offload_opts[] = {
	{.name = "hw", .id = O_HW, .type = XTTYPE_NONE},
	XTOPT_TABLEEND,
};

static void offload_parse(struct xt_option_call *cb)
{
	struct xt_flowoffload_target_info *info = cb->data;

	xtables_option_parse(cb);
	switch (cb->entry->id) {
	case O_HW:
		info->flags |= XT_FLOWOFFLOAD_HW;
		break;
	}
}

static void offload_print(const void *ip, const struct xt_entry_target *target, int numeric)
{
	const struct xt_flowoffload_target_info *info =
		(const struct xt_flowoffload_target_info *)target->data;

	printf(" FLOWOFFLOAD");
	if (info->flags & XT_FLOWOFFLOAD_HW)
		printf(" hw");
}

static void offload_save(const void *ip, const struct xt_entry_target *target)
{
	const struct xt_flowoffload_target_info *info =
		(const struct xt_flowoffload_target_info *)target->data;

	if (info->flags & XT_FLOWOFFLOAD_HW)
		printf(" --hw");
}

static struct xtables_target offload_tg_reg[] = {
	{
		.family		= NFPROTO_UNSPEC,
		.name		= "FLOWOFFLOAD",
		.revision	= 0,
		.version	= XTABLES_VERSION,
		.size		= XT_ALIGN(sizeof(struct xt_flowoffload_target_info)),
		.userspacesize	= sizeof(struct xt_flowoffload_target_info),
		.help		= offload_help,
		.print		= offload_print,
		.save		= offload_save,
		.x6_parse	= offload_parse,
		.x6_options	= offload_opts,
	},
};

void _init(void)
{
	xtables_register_targets(offload_tg_reg, ARRAY_SIZE(offload_tg_reg));
}
