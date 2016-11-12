package edu.wpi.first.wpilibj.tables;

import edu.wpi.first.wpilibj.networktables.ConnectionInfo;

/**
 * A listener that listens for connection changes in a {@link IRemote} object
 * 
 * @author Mitchell
 *
 */
public abstract class IRemoteConnectionListener {
	/**
	 * Called when an IRemote is connected
	 * @param remote the object that connected
	 */
	public abstract void connected(IRemote remote);
	/**
	 * Called when an IRemote is disconnected
	 * @param remote the object that disconnected
	 */
	public abstract void disconnected(IRemote remote);
	/**
	 * Extended version of connected called when an IRemote is connected.
 	 * Contains the connection info of the connected remote
	 * @param remote the object that connected
	 * @param info the connection info for the connected remote
	 */
	public void connectedEx(IRemote remote, ConnectionInfo info) {
		connected(remote);
	}
	/**
	 * Extended version of connected called when an IRemote is disconnected.
	 * Contains the connection info of the disconnected remote
	 * @param remote the object that disconnected
	 * @param info the connection info for the disconnected remote
	 */
	public void disconnectedEx(IRemote remote, ConnectionInfo info) {
		disconnected(remote);
	}
}
