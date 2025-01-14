/****************************************************************************
 * examples/pwfb/pwfb_main.c
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/boardctl.h>

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sched.h>
#include <pthread.h>
#include <fixedmath.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>

#ifdef CONFIG_VNCSERVER
#  include <nuttx/video/vnc.h>
#endif

#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxtk.h>
#include <nuttx/nx/nxbe.h>
#include <nuttx/nx/nxfonts.h>

#include "pwfb_internal.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const char g_wndomsg1[] = "NuttX is cool!";
static const char g_wndomsg2[] = "NuttX is fun!";
static const char g_wndomsg3[] = "NuttX is groovy!";

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pwfb_server_initialize
 ****************************************************************************/

static bool pwfb_server_initialize(FAR struct pwfb_state_s *st)
{
  struct sched_param param;
  int ret;

  /* Set the client task priority */

  param.sched_priority = CONFIG_EXAMPLES_PWFB_CLIENT_PRIO;
  ret = sched_setparam(0, &param);
  if (ret < 0)
    {
      printf("pwfb_server_initialize: ERROR: "
             "sched_setparam failed: %d\n" ,
             ret);
      return false;
    }

  /* Start the NX server kernel thread */

  ret = boardctl(BOARDIOC_NX_START, 0);
  if (ret < 0)
    {
      printf("pwfb_server_initialize: ERROR: "
             "Failed to start the NX server: %d\n",
             errno);
      return false;
    }

  /* Connect to the server */

  st->hnx = nx_connect();
  if (st->hnx)
    {
#ifdef CONFIG_VNCSERVER
      /* Setup the VNC server to support keyboard/mouse inputs */

      ret = vnc_default_fbinitialize(0, st->hnx);
      if (ret < 0)
        {
          printf("pwfb_server_initialize: ERROR: "
                 "vnc_default_fbinitialize failed: %d\n",
                 ret);
          nx_disconnect(st->hnx);
          return false;
        }
#endif
    }
  else
    {
      printf("pwfb_server_initialize: ERROR: "
             "nx_connect failed: %d\n",
             errno);
      return false;
    }

  return true;
}

/****************************************************************************
 * Name: pwfb_listener_initialize
 ****************************************************************************/

static bool pwfb_listener_initialize(FAR struct pwfb_state_s *st)
{
  struct sched_param param;
  pthread_attr_t attr;
  pthread_t thread;
  int ret;

  /* Start a separate thread to listen for server events.  This is probably
   * the least efficient way to do this, but it makes this example flow more
   * smoothly.
   */

  (void)pthread_attr_init(&attr);
  param.sched_priority = CONFIG_EXAMPLES_PWFB_LISTENER_PRIO;
  (void)pthread_attr_setschedparam(&attr, &param);
  (void)pthread_attr_setstacksize(&attr, CONFIG_EXAMPLES_PWFB_LISTENER_STACKSIZE);

  ret = pthread_create(&thread, &attr, pwfb_listener, st);
  if (ret != 0)
    {
       printf("pwfb_listener_initialize: ERROR: "
              "pthread_create failed: %d\n",
              ret);
       return false;
    }

  /* Don't return until we are connected to the server */

  while (!st->connected)
    {
      /* Wait for the listener thread to wake us up when we really
       * are connected.
       */

      (void)sem_wait(&st->semevent);
    }

  return true;
}

/****************************************************************************
 * Name: pwfb_state_initialize
 ****************************************************************************/

static bool pwfb_state_initialize(FAR struct pwfb_state_s *st)
{
  FAR const struct nx_font_s *fontset;

  /* Initialize semaphores */

  sem_init(&st->semevent, 0, 0);

  /* Initialize color information */

  st->wndo[0].color[0] = CONFIG_EXAMPLES_PWFB_COLOR1;
  st->wndo[1].color[0] = CONFIG_EXAMPLES_PWFB_COLOR2;
  st->wndo[2].color[0] = CONFIG_EXAMPLES_PWFB_COLOR3;
  st->color[0]         = CONFIG_EXAMPLES_PWFB_TBCOLOR;

  /* Connect each window to the font cache.  They cannot share the
   * font cache becuse of the differing background colors.
   */

  st->wndo[0].fcache = nxf_cache_connect(CONFIG_EXAMPLES_PWFB_FONTID,
                                         CONFIG_EXAMPLES_PWFB_FONTCOLOR,
                                         CONFIG_EXAMPLES_PWFB_COLOR1,
                                         CONFIG_EXAMPLES_PWFB_BPP, 8);
  if (st->wndo[0].fcache == NULL)
    {
      printf("pwfb_state_initialize: ERROR: "
             "Failed to connect to font cache for window 1,"
             "font ID %d: %d\n",
             CONFIG_EXAMPLES_PWFB_FONTID, errno);
      return false;
    }

  st->wndo[1].fcache = nxf_cache_connect(CONFIG_EXAMPLES_PWFB_FONTID,
                                         CONFIG_EXAMPLES_PWFB_FONTCOLOR,
                                         CONFIG_EXAMPLES_PWFB_COLOR2,
                                         CONFIG_EXAMPLES_PWFB_BPP, 8);
  if (st->wndo[1].fcache == NULL)
    {
      printf("pwfb_state_initialize: ERROR: "
             "Failed to connect to font cache for window 2,"
             "font ID %d: %d\n",
             CONFIG_EXAMPLES_PWFB_FONTID, errno);
      goto errout_with_fcache1;
    }

  st->wndo[2].fcache = nxf_cache_connect(CONFIG_EXAMPLES_PWFB_FONTID,
                                         CONFIG_EXAMPLES_PWFB_FONTCOLOR,
                                         CONFIG_EXAMPLES_PWFB_COLOR3,
                                         CONFIG_EXAMPLES_PWFB_BPP, 8);
  if (st->wndo[2].fcache == NULL)
    {
      printf("pwfb_state_initialize: ERROR: "
             "Failed to connect to font cache for window 3,"
             "font ID %d: %d\n",
             CONFIG_EXAMPLES_PWFB_FONTID, errno);
      goto errout_with_fcache2;
    }

  /* Get the handle of the font managed by the font caches.  Since the
   * font is used, the same font handle can be shared.
   */

  st->hfont = nxf_cache_getfonthandle(st->wndo[0].fcache);
  if (st->hfont == NULL)
    {
      printf("pwfb_state_initialize: ERROR: "
             "Failed to get handle for font ID %d: %d\n",
             CONFIG_EXAMPLES_PWFB_FONTID, errno);
      goto errout_with_fcache3;
    }

  /* Get information about the font set being used and save this in the
   * state structure
   */

  fontset      = nxf_getfontset(st->hfont);
  st->fheight  = fontset->mxheight;
  st->fwidth   = fontset->mxwidth;
  st->spwidth  = fontset->spwidth;

  return true;

errout_with_fcache3:
  nxf_cache_disconnect(st->wndo[2].fcache);

errout_with_fcache2:
  nxf_cache_disconnect(st->wndo[1].fcache);

errout_with_fcache1:
  nxf_cache_disconnect(st->wndo[0].fcache);
  return false;
}

/****************************************************************************
 * Name: pwfb_putc
 ****************************************************************************/

static bool pwfb_putc(FAR struct pwfb_state_s *st,
                      FAR struct pwfb_window_s *wndo,
                      FAR struct nxgl_point_s *fpos,
                      uint8_t ch)
{
  FAR const struct nxfonts_glyph_s *glyph;
  FAR const struct nx_fontbitmap_s *fbm;
  struct nxgl_rect_s bounds;
  struct nxgl_size_s fsize;
  FAR const void *src;
  int ret;

  /* Find (or create) the matching glyph */

  glyph = nxf_cache_getglyph(wndo->fcache, ch);
  if (glyph == NULL)
    {
      /* No, there is no font for this code.  Treat it like a space. */

      fpos->x += st->spwidth;
      return true;
    }

  /* Get information about the glyph? */

  fbm = nxf_getbitmap(st->hfont, ch);
  if (fbm == NULL)
    {
      /* Should not happen ceause we already know that the character has a
       * glyph.
       */

      fpos->x += st->spwidth;
      return true;
    }

  /* Get the font size */

  fsize.w = fbm->metric.width + fbm->metric.xoffset;
  fsize.h = fbm->metric.height + fbm->metric.yoffset;

  /* Construct a bounding box for the glyph */

  bounds.pt1.x = fpos->x;
  bounds.pt1.y = fpos->y;
  bounds.pt2.x = fpos->x + fsize.w - 1;
  bounds.pt2.y = fpos->y + fsize.h - 1;

  /* Render the glyph into the window */

  src = (FAR const void *)glyph->bitmap;
  ret =  nxtk_bitmapwindow(wndo->hwnd, &bounds, &src,
                           fpos, (unsigned int)glyph->stride);
  if (ret < 0)
    {
      printf("pwfb_putc: ERROR: "
             "nxtk_bitmapwindow failed: %d\n",
             errno);
      return false;
    }

  /* Set up the next character position */

  fpos->x += glyph->width;
  return true;
}

/****************************************************************************
 * Name: pwfb_configure_window
 ****************************************************************************/

static bool pwfb_configure_window(FAR struct pwfb_state_s *st, int wndx,
                                  FAR struct nxgl_size_s *size,
                                  FAR struct nxgl_point_s *pos,
                                  FAR const char *text,
                                  double deltax, double deltay)
{
  FAR struct pwfb_window_s *wndo = &st->wndo[wndx];
  FAR const char *ptr;
  struct nxgl_rect_s rect;
  struct nxgl_point_s textpos;
  nxgl_coord_t tbheight;
  int ret;

  /* Set the size of the window */

  printf("pwfb_configure_window: Set window %d size to (%d,%d)\n",
         wndx + 1, size->w, size->h);

  ret = nxtk_setsize(wndo->hwnd, size);
  if (ret < 0)
    {
      printf("pwfb_configure_window: ERROR: "
             "nxtk_setsize failed: %d\n", errno);
      goto errout_with_hwnd;
    }

  /* Set the position of window */

  printf("pwfb_configure_window: Set window %d position to (%d,%d)\n",
         wndx + 1, pos->x, pos->y);

  ret = nxtk_setposition(wndo->hwnd, pos);
  if (ret < 0)
    {
      printf("pwfb_configure_window: ERROR: "
             "nxtk_setposition failed: %d\n",
             errno);
      goto errout_with_hwnd;
    }

  /* Create a toolbar */

  tbheight = size->h >> 3;
  ret = nxtk_opentoolbar(wndo->hwnd, tbheight, &g_pwfb_tbcb, st);
  if (ret < 0)
    {
      printf("nxeq_opentoolbar: nxtk_opentoolbar failed: %d\n", errno);
    }

  /* There is a race condition here we resolve by making the main thread
   * lowest in priority.  In order for the size and position to take effect,
   * a command is sent to server which responds with an event.  So we need
   * to be synchronized at this point or the following fill will fail because
   * it depends on current knowlede of the size and position.
   */

  /* Create a bounding box.  This is actually too large because it does not
   * account for the boarder widths.  However, NX should clip the fill to
   * stay within the frame.
   */

  rect.pt1.x = 0;
  rect.pt1.y = 0;
  rect.pt2.x = size->w - 1;
  rect.pt2.y = size->h - tbheight - 1;

  /* Fill the window with the selected color */

  ret = nxtk_fillwindow(wndo->hwnd, &rect, wndo->color);
  if (ret < 0)
    {
      printf("pwfb_configure_window: ERROR: "
             "nxtk_fillwindow failed: %d\n",
             errno);
      goto errout_with_hwnd;
    }

  /* Fill the toolbar with the selected color */

  rect.pt2.y = tbheight - 1;

  ret = nxtk_filltoolbar(wndo->hwnd, &rect, st->color);
  if (ret < 0)
    {
      printf("pwfb_configure_window: ERROR: "
             "nxtk_filltoobar failed: %d\n",
             errno);
      goto errout_with_hwnd;
    }

  /* Add the text to the display, character at a time */

  textpos.x = st->spwidth;
  textpos.y = st->fheight;

  for (ptr = text; *ptr != '\0'; ptr++)
    {
      if (!pwfb_putc(st, wndo, &textpos, (uint8_t)*ptr))
        {
          printf("pwfb_configure_window: ERROR: "
                 "pwfb_putc failed\n");
          goto errout_with_hwnd;
        }
    }

  /* Set up for motion */

  wndo->xmax   = itob16(st->xres - size->w - 1);
  wndo->ymax   = itob16(st->yres - size->h - 1);
  wndo->ypos   = itob16(pos->y);
  wndo->xpos   = itob16(pos->x);
  wndo->deltax = dtob16(deltax);
  wndo->deltay = dtob16(deltay);

  return true;

errout_with_hwnd:
  printf("pwfb_configure_window: Close window %d\n", wndx + 1);

  ret = nxtk_closewindow(wndo->hwnd);
  if (ret < 0)
    {
      printf("pwfb_configure_window: ERROR: "
             "nxtk_closewindow failed: %d\n",
             errno);
    }

  return false;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pwfb_main
 ****************************************************************************/

#ifdef BUILD_MODULE
int main(int argc, FAR char *argv[])
#else
int pwfb_main(int argc, char *argv[])
#endif
{
  struct pwfb_state_s wstate;
  struct nxgl_size_s size;
  struct nxgl_point_s pos;
  nxgl_mxpixel_t color;
  int errcode = EXIT_SUCCESS;
  int ret;

  /* Connect to the NX server */

  memset(&wstate, 0, sizeof(struct pwfb_state_s));
  if (!pwfb_server_initialize(&wstate) || wstate.hnx == NULL)
    {
      printf("pwfb_main: ERROR: "
             "Failed to get NX handle\n");
      goto errout;
    }

  printf("pwfb_main: NX handle=%p\n", wstate.hnx);

  /* Start the listener thread */

  if (!pwfb_listener_initialize(&wstate))
    {
      printf("pwfb_main: ERROR: "
             "pwfb_listener_initialize failed\n");
      goto errout_with_nx;
    }

  /* Initialize the window state, colors, font cache, etc. */

  if (!pwfb_state_initialize(&wstate))
    {
      printf("pwfb_main: ERROR: "
             "pwfb_state_initialize failed\n");
      goto errout_with_nx;
    }

  /* Set the background to the configured background color */

  printf("pwfb_main: Set background color=%d\n",
         CONFIG_EXAMPLES_PWFB_BGCOLOR);

  color = CONFIG_EXAMPLES_PWFB_BGCOLOR;
  ret = nx_setbgcolor(wstate.hnx, &color);
  if (ret < 0)
    {
      printf("pwfb_main: nx_setbgcolor failed: %d\n", errno);
      goto errout_with_fontcache;
    }

  /* Open window 1 */

  printf("pwfb_main: Open window 1\n");

  wstate.wndo[0].hwnd = nxtk_openwindow(wstate.hnx, NXBE_WINDOW_RAMBACKED,
                                        &g_pwfb_wncb, (FAR void *)&wstate);
  if (wstate.wndo[0].hwnd == NULL)
    {
      printf("pwfb_main: ERROR: "
             "nxtk_openwindow failed: %d\n",
             errno);
      goto errout_with_fontcache;
    }

  printf("pwfb_main: hwnd1=%p\n", wstate.wndo[0].hwnd);

  /* Wait until we receive the screen resolution from the server.  We only
   * need to do this once after opening the first window.
   */

  while (!wstate.haveres)
    {
      (void)sem_wait(&wstate.semevent);
    }

  printf("pwfb_main: Screen resolution (%d,%d)\n",
         wstate.xres, wstate.yres);

  /* Configure window 1 */

  size.w = wstate.xres / 2;
  size.h = wstate.yres / 2;

  pos.x  = wstate.xres / 8;
  pos.y  = wstate.yres / 8;

  if (!pwfb_configure_window(&wstate, 0, &size, &pos, g_wndomsg1, 4.200, 4.285))
    {
      printf("pwfb_main: ERROR: "
             "pwfb_configure_window failed for window 1\n");
      goto errout_with_hwnd1;
    }

  /* Open window 2 */

  printf("pwfb_main: Open window 2\n");

  wstate.wndo[1].hwnd = nxtk_openwindow(wstate.hnx, NXBE_WINDOW_RAMBACKED,
                                        &g_pwfb_wncb, (FAR void *)&wstate);
  if (wstate.wndo[1].hwnd == NULL)
    {
      printf("pwfb_main: ERROR: "
             "nxtk_openwindow failed: %d\n",
             errno);
      goto errout_with_hwnd1;
    }

  printf("pwfb_main: hwnd1=%p\n", wstate.wndo[1].hwnd);

  /* Configure window 2 (same size) */

  pos.x  = wstate.xres / 4;
  pos.y  = wstate.yres / 4;

  if (!pwfb_configure_window(&wstate, 1, &size, &pos, g_wndomsg2, -3.317, 5.0))
    {
      printf("pwfb_main: ERROR: "
             "pwfb_configure_window failed for window 2\n");
      goto errout_with_hwnd2;
    }

  /* Open window 3 */

  printf("pwfb_main: Open window 3\n");

  wstate.wndo[2].hwnd = nxtk_openwindow(wstate.hnx, NXBE_WINDOW_RAMBACKED,
                                        &g_pwfb_wncb, (FAR void *)&wstate);
  if (wstate.wndo[2].hwnd == NULL)
    {
      printf("pwfb_main: ERROR: "
             "nxtk_openwindow failed: %d\n",
             errno);
      goto errout_with_hwnd2;
    }

  printf("pwfb_main: hwnd2=%p\n", wstate.wndo[2].hwnd);

  /* Configure window 3 (same size) */

  pos.x = (3 * wstate.xres) / 8;
  pos.y = (3 * wstate.yres) / 8;

  if (!pwfb_configure_window(&wstate, 2, &size, &pos, g_wndomsg3, 4.600, -3.852))
    {
      printf("pwfb_main: ERROR: "
             "pwfb_configure_window failed for window 2\n");
      goto errout_with_hwnd3;
    }

  /* Now loop animating the windows */

  for (; ; )
    {
      usleep(CONFIG_EXAMPLES_PWFB_RATECONTROL * 1000);
      if (!pwfb_motion(&wstate))
        {
          printf("pwfb_main: ERROR:"
                 "pwfb_motion failed\n");
          goto errout_with_hwnd3;
        }
    }

  errcode = EXIT_SUCCESS;

  /* Close window 3 */

errout_with_hwnd3:
  printf("pwfb_main: Close window #2\n");

  ret = nxtk_closewindow(wstate.wndo[2].hwnd);
  if (ret < 0)
    {
      printf("pwfb_main: ERROR: nxtk_closewindow failed: %d\n", errno);
    }

  /* Close window 2 */

errout_with_hwnd2:
  printf("pwfb_main: Close window #2\n");

  ret = nxtk_closewindow(wstate.wndo[1].hwnd);
  if (ret < 0)
    {
      printf("pwfb_main: ERROR: nxtk_closewindow failed: %d\n", errno);
    }

  /* Close window1 */

errout_with_hwnd1:
  printf("pwfb_main: Close window #1\n");

  ret = nxtk_closewindow(wstate.wndo[0].hwnd);
  if (ret < 0)
    {
      printf("pwfb_main: ERROR: nxtk_closewindow failed: %d\n", errno);
    }

errout_with_fontcache:
  /* Release the font cache */

  nxf_cache_disconnect(wstate.wndo[0].fcache);
  nxf_cache_disconnect(wstate.wndo[1].fcache);
  nxf_cache_disconnect(wstate.wndo[2].fcache);

errout_with_nx:
  /* Disconnect from the server */

  printf("pwfb_main: Disconnect from the server\n");
  nx_disconnect(wstate.hnx);

errout:
  return errcode;
}
