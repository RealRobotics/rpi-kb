# General Networking Issues

## Raspberry Pi Connect

This new feature from Raspberry Pi allows you to connect with any Raspberry Pi on the internet.  Full instructions can be found here: <https://www.raspberrypi.com/documentation/services/connect.html>

Once this is setup, we can connect to the Raspberry Pi using a terminal or a full screen (like VNC).

The Raspberry Pis can be accessed here: <https://connect.raspberrypi.com/devices>

## GitHub

When coding on a PC and testing on our RRLab-5G network, port 80 is blocked so we need to use port 443 instead.  This is how you set it up on the Pi (or your Linux PC).

1. [Create an SSH key](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent) for the data loggers so we can access Git.

    ```bash
    $ ssh-keygen -t ed25519 -C "x.xxx@leeds.ac.uk"
    Generating public/private ed25519 key pair.
    Enter file in which to save the key (/home/logger/.ssh/id_ed25519):
    Enter passphrase for "/home/logger/.ssh/id_ed25519" (empty for no passphrase):
    Enter same passphrase again:
    Your identification has been saved in /home/logger/.ssh/id_ed25519
    Your public key has been saved in /home/logger/.ssh/id_ed25519.pub
    The key fingerprint is:
    SHA256:... ...@leeds.ac.uk
    The key's randomart image is:
    ...
    ```

    I used no passphrase to make it easier to use.

1. Changed the name of the SSH file to make it easier to identify.

    ```bash
    cd .ssh
    mv id_ed25519 id_ed25519_ajb_data_logger
    mv id_ed25519.pub id_ed25519_ajb_data_logger.pub
    ```

1. Added this to my GitHub account following the instructions here: <https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account>

    Commands used were:

    ```bash
    eval "$(ssh-agent -s)"
    ssh-add ~/.ssh/id_ed25519_ajb_data_logger
    # Test the connection works
    ssh -T -p 443 git@ssh.github.com
    Hi AndyBlightLeeds! You've successfully authenticated, but GitHub does not provide shell access.
    ```

    To make this work on startup, I added the following to a new file `~/.ssh/config`:

    ```sshconfig
    Host ssh.github.com github.com
        Hostname ssh.github.com
        # Use port 443 for all GitHub traffic
        Port 443
        User git
        # Use specific SSH file for GitHub access
        IdentityFile ~/.ssh/id_ed25519_ajb_data_logger
        IdentitiesOnly yes

    AddKeysToAgent yes
    ```

    then chmodded it `chmod 644 config`.  Then tested with this command:

    ```bash
    $ ssh -T git@ssh.github.com
    Hi AndyBlightLeeds! You've successfully authenticated, but GitHub does not provide shell access.
    ```

    References:

    * <https://docs.github.com/en/authentication/troubleshooting-ssh/using-ssh-over-the-https-port>

## Start the SSH service on boot

There is a recurring problem where the SSH agent stops running on the RPi.  The fixes are:

Create the service file:

```bash
mkdir -p ~/.config/systemd/user
nano ~/.config/systemd/user/ssh-agent.service
```

Paste the following configuration:

```ini
[Unit]
Description=SSH key agent
[Service]
Type=simple
Environment=SSH_AUTH_SOCK=%t/ssh-agent.socket
ExecStart=/usr/bin/ssh-agent -D -a $SSH_AUTH_SOCK
[Install]
WantedBy=default.target
```

Enable and start the service:

```bash
systemctl --user daemon-reload
systemctl --user enable --now ssh-agent
```

For your terminal to find the running agent, the SSH_AUTH_SOCK variable must point to the correct socket.
For Bash: Add `export SSH_AUTH_SOCK="$XDG_RUNTIME_DIR/ssh-agent.socket"` to your `~/.bash_aliases` file.
