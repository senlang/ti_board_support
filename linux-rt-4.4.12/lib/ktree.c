/*
 * ktree.c - Routines for manipulating ktrees.
 *
 * Copyright (C) 2005 Patrick Mochel
 *
 * This file is released under the GPL v2.
 *
 * This ktree interface provides a couple of structures that wrap around
 * struct list_head to provide explicit list "head" (struct ktree) and list
 * "node" (struct ktree_node) objects. For struct ktree, a spinlock is
 * included that protects access to the actual list itself. struct
 * ktree_node provides a pointer to the ktree that owns it and a kref
 * reference count that indicates the number of current users of that node
 * in the list.
 *
 * The entire point is to provide an interface for iterating over a list
 * that is safe and allows for modification of the list during the
 * iteration (e.g. insertion and removal), including modification of the
 * current node on the list.
 *
 * It works using a 3rd object type - struct ktree_iter - that is declared
 * and initialized before an iteration. ktree_iter_next() is used to acquire the
 * next element in the list. It returns NULL if there are no more items.
 * Internally, that routine takes the ktree's lock, decrements the
 * reference count of the previous ktree_node and increments the count of
 * the next ktree_node. It then drops the lock and returns.
 *
 * There are primitives for adding and removing nodes to/from a ktree.
 * When deleting, ktree_del_node() will simply decrement the reference count.
 * Only when the count goes to 0 is the node removed from the list.
 * ktree_remove_node() will try to delete the node from the list and block until
 * it is actually removed. This is useful for objects (like devices) that
 * have been removed from the system and must be freed (but must wait until
 * all accessors have finished).
 */

#include <linux/export.h>
#include <linux/sched.h>
#include <linux/ktree.h>
#include <linux/list_sort.h>

static void knode_release(struct kref *kref);
static void __ktree_put_node(struct ktree_node *node, bool kill);

void ktree_put_node(struct ktree_node *node)
{
	if (node)
		__ktree_put_node(node, false);
}
EXPORT_SYMBOL_GPL(ktree_put_node);

static void knode_set_parent(struct ktree_node *node, struct ktree_node *parent)
{
	struct ktree_node *old_parent = node->parent;

	WARN_ON(parent && parent->deleted);
	node->parent = ktree_get_node(parent);
	ktree_put_node(old_parent);
}

void ktree_init(struct ktree *ktree, void (*get)(struct ktree_node *),
		void (*put)(struct ktree_node *))
{
	ktree->root = NULL;
	spin_lock_init(&ktree->lock);
	ktree->get = get;
	ktree->put = put;
}
EXPORT_SYMBOL_GPL(ktree_init);

static void ktree_node_init(struct ktree *ktree, struct ktree_node *parent,
			    struct ktree_node *node)
{
	node->deleted = false;
	INIT_LIST_HEAD(&node->siblings);
	INIT_LIST_HEAD(&node->children);
	kref_init(&node->refcount);
	node->ktree = ktree;
	knode_set_parent(node, parent);
	if (ktree->get)
		ktree->get(node);
}

void ktree_set_root(struct ktree *ktree, struct ktree_node *root)
{
	ktree_node_init(ktree, NULL, root);
	ktree_del_node(ktree->root);
	ktree->root = root;
}
EXPORT_SYMBOL_GPL(ktree_set_root);

void ktree_add_child_before(struct ktree_node *parent, struct ktree_node *child,
			    struct ktree_node *pos)
{
	struct ktree *ktree = parent->ktree;
	struct list_head *head;

	ktree_node_init(ktree, parent, child);

	spin_lock(&ktree->lock);
	head = pos ? &pos->siblings : &parent->children;
	list_add_tail(&child->siblings, head);
	spin_unlock(&ktree->lock);
}
EXPORT_SYMBOL_GPL(ktree_add_child_before);

void ktree_add_child_after(struct ktree_node *parent, struct ktree_node *child,
			   struct ktree_node *pos)
{
	struct ktree *ktree = parent->ktree;
	struct list_head *head;

	ktree_node_init(ktree, parent, child);

	spin_lock(&ktree->lock);
	head = pos ? &pos->siblings : &parent->children;
	list_add(&child->siblings, head);
	spin_unlock(&ktree->lock);
}
EXPORT_SYMBOL_GPL(ktree_add_child_after);

static void ktree_node_destroy(struct ktree_node *node)
{
	struct ktree_node *parent = node->parent;
	struct ktree *ktree = node->ktree;

	WARN_ON(!ktree_is_leaf(node));
	if (ktree->put)
		ktree->put(node);
	ktree_put_node(parent);
}

struct ktree_waiter {
	struct list_head list;
	struct ktree_node *node;
	struct task_struct *process;
	int woken;
};

static DEFINE_SPINLOCK(ktree_remove_lock);
static LIST_HEAD(ktree_remove_waiters);

static void knode_release(struct kref *kref)
{
	struct ktree_waiter *waiter, *tmp;
	struct ktree_node *node;
	struct ktree *ktree;

	node = container_of(kref, struct ktree_node, refcount);
	WARN_ON(!node->deleted || !node->ktree);
	ktree = node->ktree;

	assert_spin_locked(&ktree->lock);

	list_del(&node->siblings);

	if (node == ktree->root)
		ktree->root = NULL;

	spin_lock(&ktree_remove_lock);
	list_for_each_entry_safe(waiter, tmp, &ktree_remove_waiters, list) {
		if (waiter->node != node)
			continue;
		waiter->woken = 1;
		mb();  /* barrier before waking up waiting process */
		wake_up_process(waiter->process);
		list_del(&waiter->list);
	}
	spin_unlock(&ktree_remove_lock);
}

typedef void (*put_fn)(struct ktree_node *);

static bool __ktree_put_node_locked(struct ktree_node *node)
{
	return !!kref_put(&node->refcount, knode_release);
}

static void __ktree_put_node_unlock(struct ktree_node *node)
{
	struct ktree *ktree = node->ktree;

	if (__ktree_put_node_locked(node)) {
		spin_unlock(&ktree->lock);
		ktree_node_destroy(node);
		spin_lock(&ktree->lock);
	}
}

static void __ktree_put_node(struct ktree_node *node, bool kill)
{
	struct ktree *ktree = node->ktree;
	bool destroy;

	spin_lock(&ktree->lock);
	if (kill) {
		WARN_ON(node->deleted);
		node->deleted = true;
	}
	destroy = __ktree_put_node_locked(node);
	spin_unlock(&ktree->lock);
	if (destroy)
		ktree_node_destroy(node);
}

int __ktree_del_node(struct ktree_node *child, void *arg)
{
	ktree_del_node(child);
	return 0;
}

void ktree_del_node(struct ktree_node *node)
{
	if (node) {
		ktree_for_each_child(node, __ktree_del_node, NULL);
		__ktree_put_node(node, true);
	}
}
EXPORT_SYMBOL_GPL(ktree_del_node);

void ktree_remove_node(struct ktree_node *node)
{
	struct ktree_waiter waiter;

	waiter.node = node;
	waiter.process = current;
	waiter.woken = 0;

	spin_lock(&ktree_remove_lock);
	list_add(&waiter.list, &ktree_remove_waiters);
	spin_unlock(&ktree_remove_lock);

	ktree_del_node(node);

	for (;;) {
		set_current_state(TASK_UNINTERRUPTIBLE);
		if (waiter.woken)
			break;
		schedule();
	}
	__set_current_state(TASK_RUNNING);
}
EXPORT_SYMBOL_GPL(ktree_remove_node);

static struct ktree_node *ktree_get_pos(struct ktree_node *parent,
					struct list_head *pos)
{
	struct ktree_node *node = NULL;

	if (pos != &parent->children)
		node = container_of(pos, struct ktree_node, siblings);
	return ktree_get_node(node);
}

struct ktree_node *ktree_try_traverse_parent(struct ktree_node *node)
{
	struct ktree_node *next;

	next = ktree_get_parent(node);
	if (next)
		ktree_put_node(node);
	return next;
}

struct ktree_node *ktree_traverse_parent(struct ktree_node *node)
{
	struct ktree_node *next;

	next = ktree_get_parent(node);
	ktree_put_node(node);
	return next;
}

struct ktree_node *ktree_traverse_sibling(struct ktree_node *node, bool reverse)
{
	struct ktree_node *next, *parent;
	struct list_head *pos;

	parent = node->parent;
	if (!parent)
		return NULL;

	spin_lock(&node->ktree->lock);
	for (;;) {
		pos = reverse ? node->siblings.prev : node->siblings.next;
		next = ktree_get_pos(parent, pos);
		__ktree_put_node_unlock(node);
		if (!next || !next->deleted)
			break;
		node = next;
	}
	spin_unlock(&node->ktree->lock);

	return next;
}

struct ktree_node *ktree_try_traverse_sibling(struct ktree_node *node,
					      bool reverse)
{
	struct ktree_node *next;

	node = ktree_get_node(node);
	next = ktree_traverse_sibling(node, reverse);
	if (next)
		ktree_put_node(node);
	return next;
}

struct ktree_node *ktree_traverse_child(struct ktree_node *node, bool reverse)
{
	struct ktree_node *child;
	struct list_head *pos;

	spin_lock(&node->ktree->lock);
	pos = reverse ? node->children.prev : node->children.next;
	child = ktree_get_pos(node, pos);
	spin_unlock(&node->ktree->lock);

	ktree_put_node(node);
	if (child && child->deleted)
		child = ktree_traverse_sibling(child, reverse);

	return child;
}

struct ktree_node *ktree_try_traverse_child(struct ktree_node *node,
					    bool reverse)
{
	struct ktree_node *next;

	node = ktree_get_node(node);
	next = ktree_traverse_child(node, reverse);
	if (next)
		ktree_put_node(node);
	return next;
}

int ktree_for_each_child(struct ktree_node *parent,
			 int (*visitor)(struct ktree_node *child, void *arg),
			 void *arg) {
	struct ktree_node *node;
	bool reverse = false;
	int error;

	/* grab a reference that we drop as we traverse */
	node = ktree_get_node(parent);

	for (node = ktree_traverse_child(node, reverse); node;
	     node = ktree_traverse_sibling(node, reverse)) {
		error = visitor(node, arg);
		if (error) {
			ktree_put_node(node);
			return error;
		}
	}
	return 0;
}
EXPORT_SYMBOL_GPL(ktree_for_each_child);

struct ktree_sort_params {
	int	(*cmp)(struct ktree_node *a, struct ktree_node *b,
		       void *arg);
	void	*arg;
};

static int __ktree_cmp(void *arg, struct list_head *_a, struct list_head *_b)
{
	struct ktree_node *a = container_of(_a, struct ktree_node, siblings);
	struct ktree_node *b = container_of(_b, struct ktree_node, siblings);
	struct ktree_sort_params *params = arg;

	return params->cmp(a, b, params->arg);
}

void ktree_sort_children(struct ktree_node *parent,
			 int (*cmp)(struct ktree_node *a, struct ktree_node *b,
				    void *arg),
			 void *arg)
{
	struct ktree_sort_params params = { .cmp = cmp, .arg = arg };

	spin_lock(&parent->ktree->lock);
	list_sort(&params, &parent->children, __ktree_cmp);
	spin_unlock(&parent->ktree->lock);
}
EXPORT_SYMBOL_GPL(ktree_sort_children);
