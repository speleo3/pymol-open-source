'''
Session based UNDO mechanism.
'''

import os
import pickle
import psutil
import sys
import threading
import time
from pymol import _cmd

# Enum values corresponding to layer0/Session.h
SESSION_DIRTY_VIEW = 1 << 0

# Stack memory limit in MB (use 10% of total physical memory)
MEMORY_LIMIT = psutil.virtual_memory().total / 0x100000 / 10

# auto-save delay in seconds
AUTO_SAVE_DELAY = 2.0
AUTO_SAVE_ENABLED = False

# View behavior
VIEWS_STORE_NEVER = 0
VIEWS_STORE_IMPLICIT = 1
VIEWS_STORE_EXPLICIT = 2
VIEWS_STORE_SMART = 3


def log_debug(*args):
    '''Log a message for debugging'''
    print(*args, file=sys._stderr)


def log_info(*args):
    '''Log a message'''
    print(*args)


_process = psutil.Process()


def memory_usage():
    '''
    Current memory usage of the application in MB
    '''
    return _process.memory_info().rss / 0x100000


def share_equal_subtrees(tree, reference):
    '''
    For a tree-like datastructure composed of lists, dicts, and comparable
    types, substitude references in `tree` with equal items in `reference`.
    @return modified (but equal) `tree` wich may share references with
    `reference`, or `reference` if both are entirely equal.
    '''
    if type(tree) is not type(reference):
        return tree

    if tree == reference:
        return reference

    if isinstance(tree, dict):
        for key in tree:
            if key in reference:
                tree[key] = share_equal_subtrees(tree[key], reference[key])
    elif isinstance(tree, list):
        for i in range(min(len(tree), len(reference))):
            tree[i] = share_equal_subtrees(tree[i], reference[i])

    return tree


class SessionStore:
    '''
    Store the entire current PyMOL session.
    '''
    mem_after = None

    @property
    def mem_growth(self):
        if self.mem_after is None:
            self.mem_after = memory_usage()

        return self.mem_after - self.mem_before

    def __init__(self, cmd):
        self.cmd = cmd

        time_before = time.time()
        self.mem_before = memory_usage()

        self._store()

        self.time_elapsed = time.time() - time_before

    def _store(self):
        self._data = self.cmd.get_session(
                compress=0, cache=0, binary=1, version=999)

    def restore(self):
        movie_accept = lambda _, _self: not (_self.get_movie_locked() and _self.accept())
        self.cmd._pymol._session_restore_tasks.append(movie_accept)

        fn = self.cmd.get('session_file')
        self.cmd.set_session(self._data)
        if fn:
            self.cmd.set('session_file', fn, updates=0)

        self.cmd._pymol._session_restore_tasks.remove(movie_accept)

    def share(self, other):
        '''
        Do a diff between our data and other's data, and share references
        to items which compare equal.
        '''
        self._data = share_equal_subtrees(self._data, other._data)


class ViewStoreSharable(SessionStore):
    '''
    Store the camera view.

    This class can be promoted to a full session with `share()`.
    '''
    def _store(self):
        self._data = None

        with self.cmd.lockcm:
            self._data_view = _cmd.get_view(self.cmd._COb)

    def restore(self):
        if self._data is not None:
            return super(ViewStoreSharable, self).restore()

        with self.cmd.lockcm:
            _cmd.set_view(self.cmd._COb, self._data_view, 1, 0, 1)

    def share(self, other):
        self._data = dict(other._data)
        self._data['view'] = list(self._data_view)


class UndoManager:
    '''
    Undo manager which implements the stack and undo/redo commands.
    '''
    def __init__(self, _self):
        self.cmd = _self
        self._count = 0
        self._stack = []
        self._idlecount = 0
        self._idledelay = 100  # milliseconds
        self._disabled = False
        self._auto_save_thread = None
        self._auto_save_filename = os.path.expanduser('~/.pymol/auto-save.pse')
        self.set_view_behavior(VIEWS_STORE_SMART)

    def set_view_behavior(self, behavior):
        '''
        Set the view storage behavior:
        - never: Neither store nor restore any views
        - implicit: Don't store undo operations for view-only changes,
          but restore the view along with other changes
        - explicit: Store and restore every view change
        - smart: Like "implicit" if the view changed significantly, otherwise
          like "never"
        '''
        assert behavior in (VIEWS_STORE_NEVER, VIEWS_STORE_IMPLICIT,
                            VIEWS_STORE_EXPLICIT, VIEWS_STORE_SMART)
        self._view_behavior = behavior

    def _get_dirty(self):
        '''
        Get and reset the session dirty flag.
        '''
        # should be OK without lock
        return _cmd._getSessionDirty(self.cmd._COb)

    def _restore(self, operation):
        log_debug('restoring', type(operation).__name__)

        view_behavior = self._view_behavior
        if view_behavior in (VIEWS_STORE_NEVER, VIEWS_STORE_SMART):
            keep_view_op = ViewStoreSharable(self.cmd)
            keep_view_center = self.cmd.get_position()

        operation.restore()

        if view_behavior == VIEWS_STORE_SMART:
            from chempy import cpv
            extent = self.cmd.get_extent()
            displacement = cpv.distance(keep_view_center, cpv.average(*extent))
            relative_displacement = displacement / (cpv.distance(*extent) + 0.01)
            log_debug(f'view displacement {relative_displacement:.2f} (rel) '
                      f'{displacement:.2f} (abs)')
            if relative_displacement < 0.25:
                view_behavior = VIEWS_STORE_NEVER
            else:
                log_debug('restoring view (smart)')

        if view_behavior == VIEWS_STORE_NEVER:
            assert keep_view_op._data is None
            keep_view_op.restore()

        self._get_dirty()

    def _get_prev_session_store(self):
        for pos in range(self._count - 1, -1, -1):
            if isinstance(self._stack[pos], SessionStore):
                return self._stack[pos]

    def _store(self, dirty, msg=None):
        '''
        Put a state on the stack if dirty is not SESSION_DIRTY_NONE.
        '''
        if not dirty:
            return

        if dirty == SESSION_DIRTY_VIEW:
            if self._view_behavior != VIEWS_STORE_EXPLICIT:
                return

            log_debug(f'storing undo view: {msg}')
            operation = ViewStoreSharable(self.cmd)
        else:
            log_debug(f'storing undo session: {msg}')
            operation = SessionStore(self.cmd)

        # Optimize data storage
        prev_session_op = self._get_prev_session_store()
        if prev_session_op is not None:
            time_before = time.time()
            operation.share(prev_session_op)
            share_time = time.time() - time_before
            if share_time > 0.01:
                log_debug(f'sharing took {share_time:.4f} seconds')

        self._stack[self._count:] = [operation]
        self._count += 1

        assert self._count == len(self._stack)

        if operation.mem_growth > MEMORY_LIMIT / 10:
            log_debug('memory growth:', operation.mem_growth, 'MB',
                      '(stack total', self.mem_usage(), 'MB)')

        if operation.time_elapsed > 0.1:
            log_debug(f'time: {operation.time_elapsed:.4f} seconds')
            multiplier = 1000  # milliseconds with a 1:1 ratio
            if self._idledelay < operation.time_elapsed * multiplier:
                log_debug('adjusting idle delay')
                self._idledelay = operation.time_elapsed * multiplier

        self._prune_stack_by_memory_limit()

        if AUTO_SAVE_ENABLED:
            self._schedule_auto_save()

    def _schedule_auto_save(self):
        self._cancel_auto_save()
        self._auto_save_thread = threading.Timer(AUTO_SAVE_DELAY, self._auto_save)
        self._auto_save_thread.daemon = True
        self._auto_save_thread.start()

    def _cancel_auto_save(self):
        if self._auto_save_thread is not None:
            self._auto_save_thread.cancel()

    def _auto_save(self):
        if self._count < 1:
            return

        time_before = time.time()

        op = self._stack[self._count - 1]
        assert isinstance(op, SessionStore)

        current_session_filename = self.cmd.get('session_file')
        if current_session_filename:
            nstrip = 2 if current_session_filename.endswith('.gz') else 1
            self._auto_save_filename = (
                current_session_filename.rsplit('.', nstrip)[0] +
                '-auto-save.pse')

        try:
            with open(self._auto_save_filename, 'wb') as handle:
                pickle.dump(op._data, handle)
        except OSError as e:
            print('auto-save failed:', e)
            return

        time_elapsed = time.time() - time_before
        log_debug(f'auto-save to {self._auto_save_filename}'
                  f' took {time_elapsed:.4f} seconds')

    def _prune_stack_by_memory_limit(self):
        '''
        Reduce stack size if it exceeds the memory limit
        '''
        mem = 0

        for drop in range(self._count, 0, -1):
            mem += self._stack[drop - 1].mem_growth

            if mem < MEMORY_LIMIT:
                continue

            log_info(
                'undo stack exceeded memory limit, '
                f'dropping {drop} operations (keeping {self._count - drop})')
            self._stack[:drop] = []
            self._count = len(self._stack)

            if self._count == 0:
                log_info(
                    'single operation exceeded memory limit, disabling undo')
                self._disabled = True

            return

    def idle_reset(self):
        self._idlecount = 0

    def idle(self, milliseconds):
        '''
        This method should be called from the GUI event loop when it is idle.
        May store an "undo" state if the session is dirty, but may also skip
        calls to avoid excessive undo states.
        '''
        if self._disabled:
            return

        if self._idlecount < self._idledelay:
            self._idlecount += milliseconds
            return

        self._idlecount = 0
        self._store(*self._get_dirty())

    def undo(self):
        '''
        The "undo" command.
        '''
        if self._count < 2:
            log_info('Nothing to undo')
            return

        self._restore(self._stack[self._count - 2])
        self._count -= 1

    def redo(self):
        '''
        The "redo" command.
        '''
        if self._count == len(self._stack):
            log_info('Nothing to redo')
            return

        self._restore(self._stack[self._count])
        self._count += 1

    def mem_usage(self):
        '''
        Estimated memory usage of the undo stack
        '''
        return sum(operation.mem_growth for operation in self._stack)
