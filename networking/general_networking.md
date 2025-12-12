# General Networking Issues

## GitHub access over port 443

We have a router that is connected to the internet using port 443 only.  To access GitHub using SSH keys (normally done on port 22), we can tell SSH to use port 443 in two ways.

1. For a one off clone of a repo, you can use:

    ```bash
    git clone ssh://git@ssh.github.com:443/YOUR-USERNAME/YOUR-REPOSITORY.git
    ```

2. For much easier access, you can use add the following to your `~/.ssh/config` file.

    ```text
    Host github.com
        Hostname ssh.github.com
        Port 443
        User git
    ```

    This force all SSH connections to `GitHub.com` to use port 443.

References:

* <https://docs.github.com/en/authentication/troubleshooting-ssh/using-ssh-over-the-https-port>
