import { MAX_SLOTS, T, makeRoomCode, ROOM_ALPHABET } from '../shared/protocol.js';

/**
 * Room registry. A room is created by the PC host and joined by up to
 * MAX_SLOTS phone controllers. Slot numbers are stable while a controller
 * is connected and are freed on disconnect.
 */
export class Rooms {
  constructor() {
    this.map = new Map(); // code -> room
  }

  create() {
    let code = makeRoomCode();
    while (this.map.has(code)) code = makeRoomCode();
    const room = {
      code,
      host: null, // host session
      controllers: new Map(), // slot -> controller session
      createdAt: Date.now(),
    };
    this.map.set(code, room);
    return room;
  }

  get(code) {
    return this.map.get(String(code || '').toUpperCase()) || null;
  }

  destroy(room) {
    this.map.delete(room.code);
    for (const c of room.controllers.values()) {
      this.safeSend(c.ws, { t: T.ERROR, code: 'room_closed', message: 'The PC closed the room.' });
      try { c.ws.close(1000, 'room_closed'); } catch { /* ignore */ }
    }
    room.controllers.clear();
    room.host = null;
  }

  freeSlot(room) {
    for (let s = 1; s <= MAX_SLOTS; s++) {
      if (!room.controllers.has(s)) return s;
    }
    return null;
  }

  lobbyPayload(room) {
    const slots = [];
    for (let s = 1; s <= MAX_SLOTS; s++) {
      const c = room.controllers.get(s);
      slots.push({
        slot: s,
        connected: !!c,
        name: c ? c.name : '',
        device: c ? c.device : '',
        battery: c ? c.battery : 65535,
        rtt: c ? Math.round(c.rtt) : 0,
      });
    }
    return { t: T.LOBBY, code: room.code, slots };
  }

  safeSend(ws, obj) {
    if (ws && ws.readyState === 1 /* OPEN */) {
      try { ws.send(JSON.stringify(obj)); } catch { /* ignore */ }
    }
  }

  sendLobby(room) {
    this.safeSend(room.host?.ws, this.lobbyPayload(room));
  }
}

export { ROOM_ALPHABET };
