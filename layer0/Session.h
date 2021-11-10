/*
 * Session-dirty mechanism
 */

#pragma once

#include "PyMOLGlobals.h"

enum {
  SESSION_DIRTY_NONE = 0,
  SESSION_DIRTY_VIEW            = 1 << 0,
  SESSION_DIRTY_SELE            = 1 << 1,
  SESSION_DIRTY_SCENES          = 1 << 2,
  SESSION_DIRTY_GLOBAL_SETTINGS = 1 << 3,
  SESSION_DIRTY_ALL = ~SESSION_DIRTY_NONE,
};

void SessionDirtyWithLabel(char const* label, PyMOLGlobals* G,
    SessionDirtyMask what = SESSION_DIRTY_ALL);
#define SessionDirty(...) SessionDirtyWithLabel(__func__, __VA_ARGS__)

SessionDirtyMask SessionGetDirty(PyMOLGlobals* G, bool clear = true);

/**
 * RAII helper for suspend_undo
 */
class SessionScopedSuspendDirty
{
  PyMOLGlobals* m_G;
  bool m_suspended = false;

public:
  void exit();

  SessionScopedSuspendDirty(PyMOLGlobals*);
  ~SessionScopedSuspendDirty() { exit(); }
};
