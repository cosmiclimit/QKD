import random
from qiskit import QuantumCircuit, Aer, execute
from qiskit.visualization import plot_histogram


# Use the high-performance Aer simulator
simulator = Aer.get_backend('aer_simulator')

# --- Simulation Parameters ---
NUM_BITS = 100 # The total number of qubits Alice will send

# 1. Alice generates her random classical data
alice_bits = [random.randint(0, 1) for _ in range(NUM_BITS)]
# 0 for Rectilinear (+), 1 for Diagonal (x)
alice_bases = [random.randint(0, 1) for _ in range(NUM_BITS)] 

print("--- Alice's Setup ---")
print(f"Bits to send:  {alice_bits[:10]}...")
print(f"Bases for bits:{alice_bases[:10]}... (0:+, 1:x)")
def alice_encodes(bits, bases):
    """Creates a list of QuantumCircuits based on Alice's bits and bases."""
    qubits = []
    for i in range(len(bits)):
        # Create a circuit with 1 qubit and 1 classical bit
        qc = QuantumCircuit(1, 1)
        
        # Encode the bit '1' by applying an X-gate
        if bits[i] == 1:
            qc.x(0)
            
        # Encode in the diagonal basis by applying an H-gate
        if bases[i] == 1:
            qc.h(0)
        
        qubits.append(qc)
    return qubits

alice_qubits = alice_encodes(alice_bits, alice_bases)

# Let's inspect the first qubit circuit as an example
print("\nExample: Circuit for Alice's first qubit:")
print(f"Bit: {alice_bits[0]}, Basis: {'x' if alice_bases[0] else '+'}")
print(alice_qubits[0].draw(output='text'))
# Bob generates his random bases
bob_bases = [random.randint(0, 1) for _ in range(NUM_BITS)]
print("\n--- Bob's Setup ---")
print(f"Bases for measurement: {bob_bases[:10]}... (0:+, 1:x)")

def bob_measures(qubits, bases):
    """Measures a list of qubits in the bases specified."""
    measured_bits = []
    for i in range(len(qubits)):
        qc = qubits[i]
        
        # Apply H-gate if Bob wants to measure in the diagonal basis
        if bases[i] == 1:
            qc.h(0)
            
        # Measure the qubit
        qc.measure(0, 0)
        
        # Execute the circuit on the simulator
        # We run each circuit for 1 shot since it's a single measurement
        job = execute(qc, simulator, shots=1)
        result = job.result()
        counts = result.get_counts(qc)
        
        # The result is a dictionary like {'0': 1} or {'1': 1}. We get the key.
        measured_bit = int(list(counts.keys())[0])
        measured_bits.append(measured_bit)
        
    return measured_bits

bob_results = bob_measures(alice_qubits, bob_bases)
print(f"Bob's measured results: {bob_results[:10]}...")
  
def sift_keys(alice_bases, bob_bases, alice_bits, bob_bits):
    """Alice and Bob compare bases and keep only the matching bits."""
    sifted_key_alice = []
    sifted_key_bob = []
    for i in range(NUM_BITS):
        if alice_bases[i] == bob_bases[i]:
            sifted_key_alice.append(alice_bits[i])
            sifted_key_bob.append(bob_bits[i])
            
    return sifted_key_alice, sifted_key_bob

sifted_alice, sifted_bob = sift_keys(alice_bases, bob_bases, alice_bits, bob_results)

print("\n--- Post-Processing: Sifting ---")
print(f"Alice's sifted key: {sifted_alice[:20]}...")
print(f"Bob's sifted key:   {sifted_bob[:20]}...")
print(f"Length of sifted key: {len(sifted_alice)}")
def check_for_eavesdropper(key_a, key_b, sample_size=20):
    """Compare a sample of the keys to calculate the error rate (QBER)."""
    errors = 0
    # Ensure sample size is not larger than the key itself
    sample_size = min(sample_size, len(key_a))
    if sample_size == 0:
        return 0.0, [] # No bits to compare
    
    # Choose random indices to compare
    sample_indices = random.sample(range(len(key_a)), sample_size)
    
    # Track the bits that are publicly revealed for the sample
    revealed_alice_sample = []
    revealed_bob_sample = []

    for i in sample_indices:
        revealed_alice_sample.append(key_a[i])
        revealed_bob_sample.append(key_b[i])
        if key_a[i] != key_b[i]:
            errors += 1
            
    qber = errors / sample_size
    
    # Create the final key by removing the publicly revealed sample bits
    final_key_a = [key_a[i] for i in range(len(key_a)) if i not in sample_indices]
    final_key_b = [key_b[i] for i in range(len(key_b)) if i not in sample_indices]
    
    # In a real protocol, Alice and Bob would only share the QBER
    # and then discard the key if it's too high.
    # They would NOT confirm the full key's equality directly here.
    # We remove the assertion because it's precisely what we're trying to detect!
    # assert final_key_a == final_key_b # <--- REMOVE THIS LINE

    return qber, final_key_a # You can return final_key_a or final_key_b, as they are meant to be the same if secure.
qber, final_key = check_for_eavesdropper(sifted_alice, sifted_bob)

print("\n--- Security Check ---")
print(f"Quantum Bit Error Rate (QBER): {qber:.2%}")
if qber > 0.05: # Setting a 5% error threshold
    print("High QBER detected! Eavesdropping likely. ABORT.")
else:
    print("QBER is acceptable. A shared secret key is established.")
    print(f"Final shared key length: {len(final_key)}")
    print(f"Final key: {final_key[:20]}...")
def eve_intercepts_and_resends(qubits):
    """Eve measures each qubit and prepares a new one to send to Bob."""
    eve_bases = [random.randint(0, 1) for _ in range(len(qubits))]
    
    # Eve measures Alice's qubits
    eve_measured_bits = bob_measures(qubits, eve_bases)
    
    # Eve prepares and sends new qubits to Bob based on her measurements
    eves_forwarded_qubits = alice_encodes(eve_measured_bits, eve_bases)
    
    return eves_forwarded_qubits
  
def run_bb84_simulation(eavesdropper_present=False):
    print(f"\n{'='*50}")
    print(f"RUNNING SIMULATION - Eavesdropper Present: {eavesdropper_present}")
    print(f"{'='*50}\n")
    
    # 1. Alice prepares her data and qubits
    alice_bits = [random.randint(0, 1) for _ in range(NUM_BITS)]
    alice_bases = [random.randint(0, 1) for _ in range(NUM_BITS)]
    alice_qubits = alice_encodes(alice_bits, alice_bases)
    
    # Decide which qubits Bob will receive
    if eavesdropper_present:
        # 2a. Eve intercepts, measures, and resends
        print("--- Eve is intercepting the channel! ---")
        qubits_for_bob = eve_intercepts_and_resends(alice_qubits)
    else:
        # 2b. Channel is secure
        print("--- Channel is secure ---")
        qubits_for_bob = alice_qubits
        
    # 3. Bob generates bases and measures
    bob_bases = [random.randint(0, 1) for _ in range(NUM_BITS)]
    bob_results = bob_measures(qubits_for_bob, bob_bases)
    
    # 4. Sifting
    sifted_alice, sifted_bob = sift_keys(alice_bases, bob_bases, alice_bits, bob_results)
    
    # 5. Security Check
    qber, final_key = check_for_eavesdropper(sifted_alice, sifted_bob)
    
    print("\n--- SIMULATION RESULTS ---")
    print(f"Initial bits sent by Alice: {NUM_BITS}")
    print(f"Bits remaining after sifting: {len(sifted_alice)}")
    print(f"Quantum Bit Error Rate (QBER): {qber:.2%}")
    
    if qber > 0.05:
        print("CONCLUSION: High error rate detected! Eavesdropper is caught. Key is discarded.")
    else:
        print("CONCLUSION: No significant eavesdropping detected. Secure key established.")
        print(f"Final shared key length: {len(final_key)}")

# --- Run Both Scenarios ---
run_bb84_simulation(eavesdropper_present=False)
run_bb84_simulation(eavesdropper_present=True)
  



