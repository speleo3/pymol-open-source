/*
 * Session-dirty mechanism
 */

#include "os_std.h"
#include "Session.h"
#include "Setting.h"
#include "Feedback.h"

/**
 * Update the session dirty state.
 *
 * If `suspend_undo` is on, then do nothing.
 *
 * @param what Bitmask of dirty flags
 */
void SessionDirtyWithLabel(
    const char* label, PyMOLGlobals* G, SessionDirtyMask what)
{
  auto suspended = SettingGet<bool>(G, cSetting_suspend_undo);

  // To see these messages on STDERR, do:
  // PyMOL> feedback enable, session, debugging
  PRINTFD(G, FB_Session) "%s(0x%x) suspended=%d\n", __func__, what, suspended ENDFD;

  if (suspended) {
    return;
  }

  if (label && !(G->session_dirty_label &&
                   (G->session_dirty_mask & ~SESSION_DIRTY_VIEW))) {
    G->session_dirty_label = label;
  }

  G->session_dirty_mask |= what;
}

/**
 * Get the current session dirty state and clear it if requested.
 *
 * @param clear If true, clear the dirty mask
 */
SessionDirtyMask SessionGetDirty(PyMOLGlobals* G, bool clear)
{
  auto mask = G->session_dirty_mask;

  if (clear) {
    G->session_dirty_mask = SESSION_DIRTY_NONE;
  }

  return mask;
}

SessionScopedSuspendDirty::SessionScopedSuspendDirty(PyMOLGlobals* G)
    : m_G(G)
{
  if (!SettingGet<bool>(m_G, cSetting_suspend_undo)) {
    SettingSet(m_G, cSetting_suspend_undo, true);
    m_suspended = true;
  }
}

void SessionScopedSuspendDirty::exit()
{
  if (m_suspended) {
    SettingSet(m_G, cSetting_suspend_undo, false);
    m_suspended = false;
  }
}
