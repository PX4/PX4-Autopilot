#include "ftpd.h"

struct fptd_account_s
{
  uint8_t         flags;
  FAR const char *user;
  FAR const char *password;
  FAR const char *home;
}

static const struct fptd_account_s g_ftpdaccounts[] =
{
  { FTPD_ACCOUNTFLAG_SYSTEM, "root",      "abc123", NULL) },
  { FTPD_ACCOUNTFLAG_GUEST,  "ftp",       NULL,     NULL },
  { FTPD_ACCOUNTFLAG_GUEST,  "anonymous", NULL,     NULL },
};
#define NACCOUNTS (sizeof(g_ftpdaccounts) / sizeof(struct fptd_account_s))

static void ftpd_accounts(FTPD_SESSION handle)
{
  FAR onst struct fptd_account_s *account;
  int i;

  for (i = 0; i < NACCOUNTS; i++)
    {
      account = &g_ftpdaccounts[i];
      ftpd_add_user(handle, account->flags, account->user, account->password, account->home);
    }
}

int ftpd_main(int s_argc, char **s_argv)
{
  FTPD_SESSION handle;
  int ret;

  /* Bring up the network */

  ret = ftpd_netinit();
  if (ret < 0)
    {
      ndbg("Failed to initialize the network\n");
      return EXIT_FAILURE;
    }

  /* Open FTPD */

  handle = ftpd_open();
  if (!handle)
    {
      ndbg("Failed to open FTPD\n");
      return EXIT_FAILURE;
    }

  /* Configure acounts */

  (void)ftpd_accounts(handle);

  /* Then drive the FTPD server */

  while (g_ftpd_break == 0)
    {
      (void)ftpd_run(handle, 1000);
    }

  /* Close the FTPD server and exit */

  ftpd_close(handle);
  return EXIT_SUCCESS;
}

/* vim: set expandtab: */
/* End of source */
